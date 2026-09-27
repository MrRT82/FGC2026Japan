"""Robot 4 offline analysis. Never deploys gains or controls hardware.

Plots/quality only. Use rebuild_extend_model.py for mandatory static/sliding
friction identification. The v1 linear model and gain ranking are retired.
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
            report['fit_blocked'] = ('V1 linear model retired: static friction must be identified. '
                                     'Use rebuild_extend_model.py with a train/validation manifest.')
    for ax, label in zip(axes, ['degrees', 'power', 'A (sampled)', 'V (sampled)']):
        ax.set_ylabel(label); ax.grid(alpha=.3)
        if ax.lines: ax.legend(fontsize=8)
    axes[-1].set_xlabel('seconds since log origin')
    fig.tight_layout(); fig.savefig(out / 'signals.png'); plt.close(fig)
    save_json(out / 'report.json', report)
    print(out / 'report.json')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    a = sub.add_parser('analyze')
    a.add_argument('csv', type=Path); a.add_argument('--properties', type=Path)
    a.add_argument('--out', type=Path, required=True); a.add_argument('--fit', action='store_true')
    a.set_defaults(func=analyze)
    args = p.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
