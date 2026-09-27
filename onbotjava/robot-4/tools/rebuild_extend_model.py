"""CSV-first, direction-dependent stick/slip identification for robot 4.

Equivalent motor voltage = duty * measured motor-Hub voltage.
Sliding: V = kA*qdd + kV*qd + kG*cos(q+phase) + Fc(direction).
Sticking: -Fs_negative <= V - gravity(q) <= Fs_positive.
All friction quantities are effective voltage thresholds, NOT torque or Java gains.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from extend_analysis import col, load, properties, save_json


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_geometry(cad):
    if cad.get('mechanism') != 'single_rigid_joint':
        raise ValueError('Only a single rigid joint is implemented; linkages need a CAD angle/torque map')
    for k in ('sensor_scale', 'sensor_offset_deg', 'gravity_phase_deg'):
        if cad.get(k) is None or not np.isfinite(float(cad[k])):
            raise ValueError('CAD/survey angle calibration required: ' + k)
    if float(cad['sensor_scale']) == 0:
        raise ValueError('sensor_scale cannot be zero')
    if cad.get('command_sign') not in (-1, 1):
        raise ValueError('command_sign must map motor duty to positive physical angle')
    if not cad.get('source'):
        raise ValueError('Record CAD revision or measurement source')


def prepare_interval(rows, entry, cad, voltage_signal):
    """Strict raw timestamps, causal voltage sample hold, no bridging missing data."""
    selected = [r for r in rows if entry['start_s'] <= float(r['angle_t_ms']) / 1000 <= entry['end_s']]
    if len(selected) < 30 or any(r['state'] != 'ACTIVE' or r.get('fault') for r in selected):
        raise ValueError('Interval must contain >=30 complete, fault-free ACTIVE rows')
    ta, tu = col(selected, 'angle_t_ms') / 1000, col(selected, 'output_t_ms') / 1000
    if (not np.isfinite(np.r_[ta, tu]).all() or np.any(np.diff(ta) <= 0)
            or np.any(np.diff(ta) > .1) or np.any(np.diff(tu) <= 0)
            or np.any(tu < ta) or np.any(tu - ta > .1)):
        raise ValueError('Timestamp gap, reversal, or excessive command latency')
    qt = col(selected, 'angle_deg') * cad['sensor_scale'] + cad['sensor_offset_deg']
    duty = col(selected, 'sent_out') * cad['command_sign']
    if not np.isfinite(np.r_[qt, duty]).all() or np.any(np.abs(duty) > 1):
        raise ValueError('Invalid angle or command')
    # Include failed samples: never hold the last valid reading across an error.
    seqs, samples = set(), []
    for r in rows:
        seq = float(r[voltage_signal + '_seq'])
        if not np.isfinite(seq) or seq <= 0 or seq in seqs:
            continue
        seqs.add(seq)
        age = float(r[voltage_signal + '_age_ms'])
        stamp = (float(r['t_ms']) - age) / 1000
        val = float(r[voltage_signal])
        duration = float(r[voltage_signal + '_read_ms'])
        valid = r[voltage_signal + '_ok'] == '1' and 0 <= age <= 250 and 0 <= duration <= 100 and 0 < val < 30
        samples.append((stamp, val if valid else np.nan))
    z = np.array(samples)
    if len(z) < 2 or not np.isfinite(z[:, 0]).all() or np.any(np.diff(z[:, 0]) <= 0):
        raise ValueError('Missing or unordered motor-Hub voltage samples')
    t = np.arange(max(ta[0], tu[0]), min(ta[-1], tu[-1]), .02)
    vi = np.searchsorted(z[:, 0], t, side='right') - 1
    if np.any(vi < 0): raise ValueError('No preceding voltage sample')
    volts = z[vi, 1]
    if not np.isfinite(volts).all() or np.any(t - z[vi, 0] > .25):
        raise ValueError('Invalid/stale voltage within interval')
    u = duty[np.searchsorted(tu, t, side='right') - 1] * volts
    return dict(t=t, q=np.interp(t, ta, qt), u=u, volts=volts, entry=entry)


def derivatives(run):
    t, q = run['t'], run['q']
    v, a = np.full(len(t), np.nan), np.full(len(t), np.nan)
    for i in range(5, len(t) - 5):
        s = slice(i - 5, i + 6)
        c = np.polynomial.polynomial.polyfit(t[s] - t[i], q[s], 3)
        v[i], a[i] = c[1], 2 * c[2]
    return v, a


def gravity_voltage(model, q):
    return model['kG_V'] * np.cos(np.deg2rad(q + model['phase_deg']))


def fit_sliding(runs, phase, moving_dps):
    xs, ys, angles, velocities = [], [], [], []
    for run in runs:
        v, a = derivatives(run)
        # Exclude reversals/near-zero windows: static friction is fitted separately.
        mask = np.isfinite(a) & (np.abs(v) >= moving_dps)
        for i in np.flatnonzero(mask):
            if not np.all(v[max(0, i-5):i+6] * np.sign(v[i]) >= moving_dps): mask[i] = False
        q, v, a, u = run['q'][mask], v[mask], a[mask], run['u'][mask]
        xs.extend(np.c_[a, v, np.cos(np.deg2rad(q + phase)), (v > 0).astype(float), -(v < 0).astype(float)])
        ys.extend(u); angles.extend(q); velocities.extend(v)
    x, y, vel = np.asarray(xs), np.asarray(ys), np.asarray(velocities)
    if len(y) < 100 or np.sum(vel > 0) < 40 or np.sum(vel < 0) < 40:
        raise ValueError('Need sufficient sliding samples in both directions, away from zero velocity')
    scale = np.sqrt(np.mean(x*x, axis=0))
    if np.any(scale == 0) or np.linalg.matrix_rank(x) < 5 or np.linalg.cond(x/scale) > 100:
        raise ValueError('Cannot separate inertia, gravity and friction: add angles/speeds/accelerations')
    coeff = np.linalg.lstsq(x, y, rcond=None)[0]
    if coeff[0] <= 0 or np.any(coeff[1:] < 0):
        raise ValueError('Nonphysical fit: inspect calibration, delay, excitation and model structure')
    residual = y - x @ coeff
    model = dict(zip(['kA_V_per_deg_s2', 'kV_V_per_deg_s', 'kG_V', 'Fc_positive_V', 'Fc_negative_V'], map(float, coeff)))
    model.update(phase_deg=phase, moving_dps=moving_dps,
                 sliding_rmse_V=float(np.sqrt(np.mean(residual**2))), condition=float(np.linalg.cond(x/scale)),
                 angle_range_deg=[float(min(angles)), float(max(angles))],
                 speed_range_dps=[float(min(vel)), float(max(vel))])
    return model


def sliding_residuals(runs, model):
    """Directional speed/angle residuals, including low speed, for the next rebuild."""
    data = []
    for run in runs:
        v, a = derivatives(run)
        valid = np.isfinite(a) & (np.abs(v) > .2)
        q, v, a, u = run['q'][valid], v[valid], a[valid], run['u'][valid]
        predicted = (model['kA_V_per_deg_s2'] * a + model['kV_V_per_deg_s'] * v
                     + gravity_voltage(model, q) + np.where(v > 0, model['Fc_positive_V'], -model['Fc_negative_V']))
        data.extend(zip(q, v, u - predicted))
    z = np.array(data)
    if len(z) < 40: raise ValueError('Need held-out sliding data for speed/angle residual checks')
    for direction in (-1, 1):
        low_speed = (z[:, 1]*direction > 0) & (np.abs(z[:, 1]) < 2)
        sliding = (z[:, 1]*direction > 0) & (np.abs(z[:, 1]) >= 2)
        if np.sum(low_speed) < 10 or np.sum(sliding) < 40:
            raise ValueError('Need held-out low-speed and sliding coverage in both directions; investigate Stribeck region')
    bins = []
    for direction in (-1, 1):
        for lo, hi in ((.2, 2), (2, 5), (5, 15), (15, float('inf'))):
            m = (z[:, 1]*direction > 0) & (np.abs(z[:, 1]) >= lo) & (np.abs(z[:, 1]) < hi)
            if np.any(m):
                bins.append(dict(direction=direction, speed_bin_dps=[lo, hi if np.isfinite(hi) else None],
                                 samples=int(np.sum(m)), mean_residual_V=float(np.mean(z[m, 2])),
                                 rms_residual_V=float(np.sqrt(np.mean(z[m, 2]**2)))))
    return bins, z


def breakaway_bracket(run, model, settings):
    e, t, q, u = run['entry'], run['t'], run['q'], run['u']
    direction = e['direction']
    if direction not in (-1, 1): raise ValueError('Breakaway direction must be +/-1')
    still, moving = float(e['last_still_s']), float(e['first_motion_s'])
    if not (t[0] + .15 <= still < moving <= t[-1] - .1) or moving - still > .3:
        raise ValueError('Breakaway annotation needs 150ms prehold, <=300ms onset bracket, 100ms postmotion')
    stationary = (t >= still - .15) & (t <= still)
    motion = (t >= moving) & (t <= moving + .1)
    if np.ptp(q[stationary]) > settings['position_noise_deg']:
        raise ValueError('Annotated pre-breakaway interval is not stationary')
    origin = float(np.median(q[stationary]))
    if np.min(direction * (q[motion] - origin)) < settings['motion_displacement_deg']:
        raise ValueError('Onset must show sustained displacement above measurement noise')
    s = (t >= still) & (t <= moving)
    net = direction * (u[s] - gravity_voltage(model, q[s]))
    if len(net) < 2 or np.min(net) <= 0 or np.any(np.diff(net) < -.03):
        raise ValueError('Breakaway must use a monotonic, correctly directed effort ramp')
    if np.max(np.abs(np.diff(u[s]) / np.diff(t[s]))) > settings['max_ramp_V_per_s']:
        raise ValueError('Effort changes too quickly to bracket quasi-static breakaway')
    uncertainty = settings['effort_uncertainty_V']
    return [float(max(0, net[0] - uncertainty)), float(net[-1] + uncertainty)]


def fit_static(runs, model, settings):
    details = []
    for direction, name in ((1, 'positive'), (-1, 'negative')):
        brackets = [breakaway_bracket(r, model, settings) for r in runs if r['entry']['direction'] == direction]
        if len(brackets) < 2:
            raise ValueError('Need >=2 independent breakaway events per direction')
        lo = max(b[0] for b in brackets)
        hi = min(b[1] for b in brackets)
        fc = model['Fc_' + name + '_V']
        lo = max(lo, fc)
        if lo > hi:
            raise ValueError('Static thresholds inconsistent across events or below dynamic friction; rebuild by angle/load, inspect Stribeck/backlash')
        model['Fs_' + name + '_interval_V'] = [lo, hi]
        model['Fs_' + name + '_V'] = (lo + hi) / 2
        details.append(dict(direction=direction, brackets_V=brackets))
    return details


def advance(q, v, u, dt, model):
    """Stick/slip switch; exact zero velocity holds until breakaway threshold.

    At a velocity zero crossing stop for the remainder of this small substep;
    evaluate opposite breakaway on the next substep (prevents friction chatter).
    """
    drive = u - gravity_voltage(model, q)
    if abs(v) < 1e-9:
        if -model['Fs_negative_V'] <= drive <= model['Fs_positive_V']:
            return q, 0.
        direction = 1 if drive > 0 else -1
    else:
        direction = 1 if v > 0 else -1
    fc = model['Fc_positive_V'] if direction > 0 else model['Fc_negative_V']
    acc = (drive - model['kV_V_per_deg_s'] * v - direction * fc) / model['kA_V_per_deg_s2']
    next_v = v + acc * dt
    if v != 0 and next_v * v <= 0:
        crossing = -v / acc
        return q + v * crossing + .5 * acc * crossing**2, 0.
    return q + v * dt + .5 * acc * dt**2, next_v


def replay(run, model, settings):
    t, actual, u = run['t'], run['q'], run['u']
    if np.ptp(actual[:8]) > settings['position_noise_deg']:
        raise ValueError('Validation replay must start with at least 160ms stationary')
    if np.ptp(actual[-15:]) > settings['position_noise_deg']:
        raise ValueError('Validation replay must include at least 300ms final hold')
    q, v = float(np.mean(actual[:8])), 0.
    predicted = [q]
    for i, delta in enumerate(np.diff(t)):
        for _ in range(20):
            q, v = advance(q, v, u[i], delta / 20, model)
        if not np.isfinite([q, v]).all() or abs(q - actual[0]) > 180:
            raise ValueError('Free replay diverged')
        predicted.append(q)
    residual = np.array(predicted) - actual
    rmse, peak = float(np.sqrt(np.mean(residual**2))), float(np.max(np.abs(residual)))
    def onset(series):
        moving = np.abs(np.array(series) - np.mean(series[:8])) >= settings['motion_displacement_deg']
        for j in range(len(moving) - 4):
            if np.all(moving[j:j+5]): return float(t[j])
        return None
    measured_onset, predicted_onset = onset(actual), onset(predicted)
    onset_error = abs(predicted_onset - measured_onset) if measured_onset is not None and predicted_onset is not None else None
    hold_drift = float(np.ptp(predicted[-15:]))
    return dict(rmse_deg=rmse, max_error_deg=peak,
                onset_error_s=onset_error, final_hold_drift_deg=hold_drift,
                pass_angle=rmse <= settings['max_replay_rmse_deg'] and peak <= settings['max_replay_error_deg']
                and onset_error is not None and onset_error <= settings['max_onset_error_s']
                and hold_drift <= settings['position_noise_deg']), np.array(predicted)


def rebuild(manifest_path, out):
    out.mkdir(parents=True, exist_ok=True)
    report = dict(schema='robot4-stick-slip-v2', status='needs_data', gain_generation_enabled=False,
                  inputs=[], next_iteration=[])
    # Invalidate an older successful run before parsing/fitting a new revision.
    save_json(out / 'model_report.json', report)
    try:
        spec = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
        report['manifest_sha256'] = digest(manifest_path)
        cad = spec['geometry']; check_geometry(cad)
        report['geometry'] = cad
        report['load'] = dict(balls=spec['balls'], motor_model=spec['motor_model'], gear_ratio=spec['gear_ratio'])
        if not isinstance(spec.get('motor_hub_voltage_signal'), str) or not spec['motor_hub_voltage_signal'].endswith('_voltage_V'):
            raise ValueError('Specify the voltage column of the Hub physically powering the extension motor')
        settings = spec['identification']
        numeric = ['moving_dps', 'position_noise_deg', 'motion_displacement_deg', 'max_ramp_V_per_s',
                   'effort_uncertainty_V', 'max_replay_rmse_deg', 'max_replay_error_deg', 'max_sliding_rmse_V', 'max_onset_error_s']
        if any(not np.isfinite(float(settings[k])) or float(settings[k]) <= 0 for k in numeric):
            raise ValueError('Identification tolerances must be finite and positive')
        if settings['motion_displacement_deg'] <= settings['position_noise_deg']:
            raise ValueError('Motion displacement must exceed position noise')
        runs, seen_roles, all_intervals = [], {}, {}
        for file in spec['files']:
            path = (manifest_path.parent / file['csv']).resolve()
            meta_path = path.with_suffix('.properties')
            meta = properties(meta_path)
            if meta.get('robot') != '4' or meta.get('balls') != str(spec['balls']) or int(spec['balls']) < 0:
                raise ValueError('Robot/load metadata mismatch or unknown ball count')
            if meta.get('gear_ratio') != spec['gear_ratio'] or meta.get('motor_model') != spec['motor_model']:
                raise ValueError('Motor/gearing metadata mismatch')
            rows, rejected = load(path)
            if rejected: raise ValueError('Incomplete CSV: choose intact trial file; do not bridge missing rows')
            key = digest(path)
            if file['role'] not in ('train', 'validation'): raise ValueError('Invalid dataset role')
            if key in seen_roles and seen_roles[key] != file['role']:
                raise ValueError('Validation must use a different CSV recording; content hash reused')
            seen_roles[key] = file['role']
            report['inputs'].append(dict(csv=str(path), sha256=key, metadata_sha256=digest(meta_path), role=file['role']))
            for entry in file['intervals']:
                if entry['kind'] not in ('sliding', 'breakaway', 'replay'): raise ValueError('Unknown interval kind')
                if entry['end_s'] <= entry['start_s']: raise ValueError('Invalid interval bounds')
                previous = all_intervals.setdefault(key, [])
                if any(max(entry['start_s'], a) < min(entry['end_s'], b) for a, b in previous):
                    raise ValueError('Overlapping intervals would duplicate evidence')
                previous.append((entry['start_s'], entry['end_s']))
                run = prepare_interval(rows, entry, cad, spec['motor_hub_voltage_signal'])
                run['role'] = file['role']; runs.append(run)
        select = lambda role, kind: [r for r in runs if r['role'] == role and r['entry']['kind'] == kind]
        model = fit_sliding(select('train', 'sliding'), cad['gravity_phase_deg'], settings['moving_dps'])
        train_volts = np.concatenate([r['volts'] for r in runs if r['role'] == 'train'])
        train_effort = np.concatenate([r['u'] for r in runs if r['role'] == 'train'])
        model['voltage_range_V'] = [float(min(train_volts)), float(max(train_volts))]
        model['effort_range_V'] = [float(min(train_effort)), float(max(train_effort))]
        report['model'] = model
        report['breakaway_training'] = fit_static(select('train', 'breakaway'), model, settings)
        if model['sliding_rmse_V'] > settings['max_sliding_rmse_V']:
            raise ValueError('Sliding residual too large; inspect residual-vs-speed/angle before adding Stribeck or angle-dependent friction')
        results = []
        report['validation'] = results
        bins, residual = sliding_residuals(select('validation', 'sliding'), model)
        report['sliding_validation_bins'] = bins
        results.append(dict(kind='sliding_residual', passed=all(b['rms_residual_V'] <= settings['max_sliding_rmse_V'] for b in bins)))
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for ax, j, label in ((axes[0], 1, 'speed (deg/s)'), (axes[1], 0, 'angle (deg)')):
            ax.scatter(residual[:, j], residual[:, 2], s=3); ax.axhline(0, color='black')
            ax.set(xlabel=label, ylabel='voltage residual (V)')
        fig.tight_layout(); fig.savefig(out / 'sliding_residuals.png'); plt.close(fig)
        validation_breakaway = select('validation', 'breakaway')
        for direction, name in ((1, 'positive'), (-1, 'negative')):
            events = [r for r in validation_breakaway if r['entry']['direction'] == direction]
            if not events: raise ValueError('Held-out breakaway event required in each direction')
            for event in events:
                bracket = breakaway_bracket(event, model, settings)
                lo, hi = model['Fs_' + name + '_interval_V']
                results.append(dict(kind='breakaway', direction=direction, bracket_V=bracket,
                                    passed=max(lo, bracket[0]) <= min(hi, bracket[1])))
        replay_runs = select('validation', 'replay')
        if len(replay_runs) < 2: raise ValueError('Need two held-out rest-to-motion-to-stop replays, both directions')
        directions = {int(np.sign(r['q'][np.argmax(np.abs(r['q'] - r['q'][0]))] - r['q'][0])) for r in replay_runs}
        if not {-1, 1}.issubset(directions): raise ValueError('Validation replays must cover both directions')
        for i, run in enumerate(replay_runs):
            if np.min(run['q']) < model['angle_range_deg'][0] or np.max(run['q']) > model['angle_range_deg'][1]:
                raise ValueError('Validation angles outside trained coverage: collect matching sliding data')
            nominal, prediction = replay(run, model, settings)
            sensitivity = []
            for pos, neg in itertools.product(model['Fs_positive_interval_V'], model['Fs_negative_interval_V']):
                alternative = dict(model, Fs_positive_V=pos, Fs_negative_V=neg)
                metrics, _ = replay(run, alternative, settings); sensitivity.append(metrics)
            results.append(dict(kind='replay', nominal=nominal, static_threshold_corners=sensitivity,
                                passed=nominal['pass_angle'] and all(m['pass_angle'] for m in sensitivity)))
            fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            axes[0].plot(run['t'], run['q'], label='CSV'); axes[0].plot(run['t'], prediction, label='stick/slip replay')
            axes[0].set_ylabel('physical angle (deg)'); axes[0].legend()
            axes[1].plot(run['t'], prediction - run['q']); axes[1].set(xlabel='log time (s)', ylabel='error (deg)')
            fig.tight_layout(); fig.savefig(out / f'replay_{i}.png'); plt.close(fig)
        report['validation'] = results
        report['status'] = 'model_validation_passed' if all(r['passed'] for r in results) else 'rebuild_required'
        report['next_iteration'] = (['Review fit and residuals; design/validate direction-dependent friction compensation before PID gain search.']
            if report['status'] == 'model_validation_passed' else
            ['Add new TRAIN recordings at failed direction/angle/speed; retain untouched independent VALIDATION recordings.',
             'Inspect stick/slip transition, backlash, delay and low-speed Stribeck residual; revise structure only with evidence.'])
    except (ValueError, KeyError, OSError, TypeError) as e:
        if 'model' in report: report['status'] = 'rebuild_required'
        report['reason'] = str(e)
        report['next_iteration'] = ['Supply/correct the indicated calibration or CSV experiments; rerun the manifest. No gains generated.']
    save_json(out / 'model_report.json', report)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest', type=Path); p.add_argument('--out', type=Path, required=True)
    a = p.parse_args(); result = rebuild(a.manifest, a.out)
    print(result['status'], result.get('reason', ''))


if __name__ == '__main__': main()
