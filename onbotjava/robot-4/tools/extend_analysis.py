"""Robot 4 offline analysis. Never deploys gains or controls hardware.

Local model: angle'' = b * power - damping * angle' + bias.
Closed-loop identification is exploratory, not proof of stability.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from analyze_extend_tune import summarize


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def properties(path):
    result = {}
    for line in Path(path).read_text(encoding='utf-8-sig').splitlines():
        if '=' in line and not line.lstrip().startswith(('#', '!')):
            key, value = line.split('=', 1)
            result[key.strip()] = value.strip()
    return result


def load(path):
    rows, rejected = [], 0
    with open(path, encoding='utf-8-sig', newline='') as f:
        for row in csv.DictReader(f):
            if None in row or any(v is None for v in row.values()):
                rejected += 1
            else:
                rows.append(row)
    if not rows:
        raise ValueError('No complete rows')
    return rows, rejected


def col(rows, key):
    def number(r):
        try:
            return float(r.get(key, 'nan'))
        except (TypeError, ValueError):
            return np.nan
    return np.array([number(r) for r in rows])


def electrical(rows, name):
    """Recover actual read-end timestamps; never count repeated cached samples."""
    result, seen = [], set()
    for r in rows:
        try:
            seq = int(r[name + '_seq'])
            if seq <= 0 or seq in seen:
                continue
            seen.add(seq)
            t = (float(r['t_ms']) - float(r[name + '_age_ms'])) / 1000
            v = float(r[name])
            age = float(r[name + '_age_ms'])
            if r[name + '_ok'] == '1' and age >= 0 and np.isfinite([t, v]).all():
                result.append((t, v))
        except (ValueError, KeyError):
            continue
    return np.array(result).reshape(-1, 2)


def trials(rows):
    """Split ACTIVE runs at faults, invalid rows and timing gaps; never bridge them."""
    runs, current, last = [], [], None
    for r in rows:
        vals = col([r], 'angle_t_ms')[0], col([r], 'angle_deg')[0], col([r], 'sent_out')[0], col([r], 'output_t_ms')[0]
        valid = r.get('state') == 'ACTIVE' and not r.get('fault') and np.isfinite(vals).all()
        t = vals[0] / 1000
        if not valid or (last is not None and not 0 < t - last <= .1):
            if current:
                runs.append(current)
            current, last = [], None
        if valid:
            current.append(r)
            last = t
    if current:
        runs.append(current)
    return runs


def prepare(run, dt=.02):
    ta = col(run, 'angle_t_ms') / 1000
    tu = col(run, 'output_t_ms') / 1000
    if np.any(np.diff(tu) <= 0):
        raise ValueError('Non-monotonic output timestamps')
    t = np.arange(max(ta[0], tu[0]), min(ta[-1], tu[-1]), dt)
    if len(t) < 60:
        raise ValueError('Need at least 1.2 seconds of continuous ACTIVE data')
    angle = np.interp(t, ta, col(run, 'angle_deg'))
    # Zero-order hold: power remains at the last issued command.
    u = col(run, 'sent_out')[np.searchsorted(tu, t, side='right') - 1]
    return t, angle, u


def features(t, angle, u):
    """Local cubic regression derivatives, 220 ms window (offline, acausal)."""
    x, y = [], []
    for j in range(5, len(t) - 5):
        sl = slice(j - 5, j + 6)
        p = np.polynomial.polynomial.polyfit(t[sl] - t[j], angle[sl], 3)
        x.append([u[j], -p[1], 1.0])
        y.append(2 * p[2])
    return np.array(x), np.array(y)


def fit_model(run):
    t, a, u = prepare(run)
    if np.ptp(a) < 3 or np.std(u) < .01:
        raise ValueError('Insufficient angle/input excitation for identification')
    x, y = features(t, a, u)
    scale = np.sqrt(np.mean(x * x, axis=0))
    if np.linalg.matrix_rank(x) < 3 or np.linalg.cond(x / scale) > 100:
        raise ValueError('Ill-conditioned model; collect independent excitation')
    coef = np.linalg.lstsq(x, y, rcond=None)[0]
    b, damping, bias = map(float, coef)
    if b <= 0 or damping < 0:
        raise ValueError('Model has wrong motor sign or negative damping')
    residual = y - x @ coef
    return dict(b=b, damping=damping, bias=bias,
                acceleration_rmse=float(np.sqrt(np.mean(residual**2))),
                angle_min=float(a.min()), angle_max=float(a.max()),
                input_min=float(u.min()), input_max=float(u.max()),
                condition=float(np.linalg.cond(x / scale)), samples=len(y))


def validate(model, run):
    t, a, u = prepare(run)
    # Free-running replay of held-out measured input, initialized once.
    position = float(a[0])
    velocity = float(np.polyfit(t[:6] - t[0], a[:6], 1)[0])
    predicted = [position]
    for n in range(len(t) - 1):
        for _ in range(10):
            h = (t[n + 1] - t[n]) / 10
            velocity += h * (model['b'] * u[n] - model['damping'] * velocity + model['bias'])
            position += h * velocity
        predicted.append(position)
    error = np.array(predicted) - a
    return dict(rmse_deg=float(np.sqrt(np.mean(error**2))), max_error_deg=float(np.max(np.abs(error)))), (t, a, predicted)


def simulate(model, cfg, kp, kd, start, step):
    """Replicate Tune PD/FF, target slew and velocity filter; I fixed at zero.

    This is not the safety state machine or an electrical model.
    """
    dt, a, v, target, previous_error, vf = .02, start, 0., start, 0., 0.
    rows = []
    limit = float(cfg['max_power'])
    for n in range(400):
        goal = start if n < 50 else start + step
        target += np.clip(goal - target, -10 * dt, 10 * dt)
        e = target - a
        friction = 0 if vf * np.sign(e) > 15 else float(cfg['friction']) * np.clip(e / 2, -1, 1)
        ff = float(cfg['gravity_cos']) * np.cos(np.deg2rad(a + float(cfg['gravity_phase_deg']))) + float(cfg['gravity_constant']) + friction
        requested = ff + kp * e + kd * (e - previous_error) / dt
        previous_error = e
        u = float(np.clip(requested, -limit, limit))
        old = a
        for _ in range(10):
            v += dt / 10 * (model['b'] * u - model['damping'] * v + model['bias'])
            a += dt / 10 * v
        vf += .35 * ((a - old) / dt - vf)
        if not np.isfinite([a, v]).all() or abs(a - start) > 30:
            return None
        rows.append([n * dt, target, a, u, requested])
    z = np.array(rows)
    if z[:, 2].min() <= float(cfg['min_angle_deg']) or z[:, 2].max() >= float(cfg['max_angle_deg']):
        return None
    if (z[:, 2].min() < model['angle_min'] or z[:, 2].max() > model['angle_max']
            or z[:, 3].min() < model['input_min'] or z[:, 3].max() > model['input_max']):
        return None
    return dict(rmse_deg=float(np.sqrt(np.mean((z[:, 1] - z[:, 2])**2))),
                saturation_fraction=float(np.mean(np.abs(z[:, 4]) > limit)),
                final_error_deg=float(abs(z[-1, 1] - z[-1, 2])))


def analyze(args):
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    rows, rejected = load(args.csv)
    cfg_path = args.properties or args.csv.with_suffix('.properties')
    cfg = properties(cfg_path)
    if cfg.get('robot') != '4':
        raise ValueError('Use the same-stem metadata snapshot: robot=4 required')
    report = dict(summary=summarize(args.csv), incomplete_rows=rejected, settings=cfg,
                  source_sha256=hashlib.sha256(args.csv.read_bytes()).hexdigest(),
                  status='exploratory; no hardware validation; no automatic deployment')
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    if 'signal' in rows[0]:
        for name in sorted({r['signal'] for r in rows}):
            valid = [r for r in rows if r['signal'] == name and r['ok'] == '1']
            ax = axes[2] if 'current' in name.lower() else axes[3]
            ax.plot(col(valid, 't_ms') / 1000, col(valid, 'value'), '.-', label=name)
    else:
        axes[0].plot(col(rows, 'angle_t_ms') / 1000, col(rows, 'angle_deg'), label='angle')
        axes[0].plot(col(rows, 't_ms') / 1000, col(rows, 'target_deg'), label='slewed target')
        axes[0].plot(col(rows, 't_ms') / 1000, col(rows, 'goal_deg'), label='operator goal', alpha=.5)
        axes[1].step(col(rows, 'output_t_ms') / 1000, col(rows, 'sent_out'), where='post', label='sent power')
        for name in [k[:-4] for k in rows[0] if k.endswith('_seq')]:
            points = electrical(rows, name)
            ax = axes[2] if 'current' in name else axes[3]
            ax.plot(points[:, 0], points[:, 1], '.-', label=name)
        runs = trials(rows)
        report['trials'] = [dict(rows=len(r), duration_s=float((col(r, 'angle_t_ms')[-1] - col(r, 'angle_t_ms')[0]) / 1000),
                                tracking_rmse_deg=float(np.sqrt(np.mean((col(r, 'target_deg') - col(r, 'angle_deg'))**2)))) for r in runs]
        if args.fit:
            try:
                eligible = [r for r in runs if len(r) >= 70]
                if len(eligible) < 2:
                    raise ValueError('Need two separate ACTIVE trials: training and held-out validation')
                train, test = eligible[0], eligible[1]
                keys = ['max_power', 'min_angle_deg', 'max_angle_deg', 'kp', 'gravity_cos',
                        'gravity_phase_deg', 'gravity_constant', 'friction']
                if not np.isfinite([float(cfg[k]) for k in keys]).all() or not 0 < float(cfg['max_power']) <= .35:
                    raise ValueError('Invalid numeric settings for candidate simulation')
                model = fit_model(train)
                metrics, replay = validate(model, test)
                report['model'], report['validation'] = model, metrics
                fig2, ax = plt.subplots(figsize=(10, 4))
                ax.plot(replay[0], replay[1], label='held-out measured')
                ax.plot(replay[0], replay[2], label='open-loop model replay')
                ax.legend(); ax.set(xlabel='seconds', ylabel='degrees')
                fig2.tight_layout(); fig2.savefig(out / 'validation.png'); plt.close(fig2)
                if metrics['rmse_deg'] > 1 or metrics['max_error_deg'] > 3:
                    raise ValueError('Held-out model mismatch: no gain ranking generated')
                start = float(np.median(col(train, 'angle_deg')))
                if start - 5 < model['angle_min'] or start + 5 > model['angle_max']:
                    raise ValueError('Training angle coverage insufficient for +/-5 degree candidate simulation')
                ranking = []
                for kp in np.linspace(0, max(.04, float(cfg['kp']) * 2), 21):
                    for kd in np.linspace(0, .01, 21):
                        scores = [simulate(model, cfg, float(kp), float(kd), start, s) for s in (-5, 5)]
                        if all(s is not None for s in scores):
                            score = max(s['rmse_deg'] + 5 * s['saturation_fraction'] + s['final_error_deg'] for s in scores)
                            ranking.append(dict(kp=float(kp), ki=0., kd=float(kd), score=score, scenarios=scores))
                report['candidate_grid_top10'] = sorted(ranking, key=lambda r: r['score'])[:10]
                report['candidate_note'] = 'PD only; FF unchanged. Local model only. Current, backlash, voltage and stalls not simulated. Human review required.'
            except ValueError as e:
                report['fit_blocked'] = str(e)
    for ax, label in zip(axes, ['degrees', 'power', 'A (sampled)', 'V (sampled)']):
        ax.set_ylabel(label); ax.grid(alpha=.3)
        if ax.lines: ax.legend(fontsize=8)
    axes[-1].set_xlabel('seconds since log origin')
    fig.tight_layout(); fig.savefig(out / 'signals.png'); plt.close(fig)
    save_json(out / 'report.json', report)
    print(out / 'report.json')


def gravity(args):
    """Estimate holding power versus angle from explicitly curated steady intervals."""
    records = []
    for item in json.loads(args.intervals.read_text(encoding='utf-8-sig')):
        rows, _ = load(args.intervals.parent / item['csv'])
        selected = [r for r in rows if item['start_s'] <= float(r['t_ms']) / 1000 <= item['end_s'] and r['state'] == 'ACTIVE']
        a, u, v = col(selected, 'angle_deg'), col(selected, 'sent_out'), col(selected, 'velocity_dps')
        if len(a) < 25 or not np.isfinite(np.r_[a, u, v]).all() or np.ptp(a) > 1 or np.max(np.abs(v)) > 2:
            raise ValueError('Interval is too short, invalid or not steady')
        records.append((float(np.mean(a)), float(np.mean(u))))
    z = np.array(records)
    if len(z) < 4 or np.ptp(z[:, 0]) < 30:
        raise ValueError('Need >=4 steady intervals spanning >=30 degrees, same load')
    x = np.c_[np.cos(np.deg2rad(z[:, 0])), np.sin(np.deg2rad(z[:, 0])), np.ones(len(z))]
    if np.linalg.cond(x) > 100:
        raise ValueError('Angle coverage ill-conditioned')
    c = np.linalg.lstsq(x, z[:, 1], rcond=None)[0]
    save_json(args.out, dict(gravity_cos=float(np.hypot(c[0], c[1])), gravity_phase_deg=float(np.rad2deg(np.arctan2(-c[1], c[0]))), gravity_constant=float(c[2]),
                            residual_power_rms=float(np.sqrt(np.mean((x @ c - z[:, 1])**2))), intervals=records,
                            note='Holding-power estimate includes static friction and controller bias; not pure gravity. Do not deploy automatically.'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    a = sub.add_parser('analyze')
    a.add_argument('csv', type=Path); a.add_argument('--properties', type=Path)
    a.add_argument('--out', type=Path, required=True); a.add_argument('--fit', action='store_true')
    a.set_defaults(func=analyze)
    g = sub.add_parser('gravity')
    g.add_argument('intervals', type=Path); g.add_argument('--out', type=Path, required=True)
    g.set_defaults(func=gravity)
    args = p.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
