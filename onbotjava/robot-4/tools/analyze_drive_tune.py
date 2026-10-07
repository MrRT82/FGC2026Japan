"""Summarise Drive_Tune_R4 CSVs per trial: acceleration, braking, turn rate and heading-step response.

Usage: python -B tools/analyze_drive_tune.py <CSV> [more CSV...] [--json]

Electrical values are multiplexed samples (deduplicated by seq). Short peaks between samples are not
seen, so a low value here does not prove the true peak was low.
"""
import argparse
import csv
import json
import math
from pathlib import Path

STOP_MPS = 0.02
STOP_DPS = 5.0
SETTLE_DEG = 3.0


def num(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def finite(values):
    return [v for v in values if not math.isnan(v)]


def load(path):
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    # A row cut by power loss has missing trailing fields (None); ignore it.
    if rows and None in rows[-1].values():
        rows = rows[:-1]
    return rows


def properties(path):
    props = {}
    p = Path(path).with_suffix('.properties')
    if p.exists():
        for line in p.read_text(encoding='utf-8').splitlines():
            if '=' in line and not line.startswith('#'):
                key, value = line.split('=', 1)
                props[key.strip()] = value.strip()
    return props


def trials(rows):
    """Rows of each trial from START through the first non-RUNNING row (its stop reason)."""
    out, current = [], None
    for r in rows:
        if r.get('event') == 'START':
            current = [r]
            out.append(current)
        elif current is not None:
            current.append(r)
            if r.get('state') != 'RUNNING':
                current = None
    return out


def samples(rows, name):
    seen, values = set(), []
    for r in rows:
        if r.get(name + '_ok') == '1' and r.get(name + '_seq') not in seen:
            seen.add(r.get(name + '_seq'))
            values.append(num(r.get(name)))
    return finite(values)


def peak(values, fn=max):
    return fn(values) if values else math.nan


def electrical(rows):
    hubs = sorted({k[:-len('_voltage_V')] for k in rows[0] if k.startswith('hub_') and k.endswith('_voltage_V')})
    current = samples(rows, 'left_current_A') + samples(rows, 'right_current_A')
    volts = [v for h in hubs for v in samples(rows, h + '_voltage_V')]
    return peak(current), peak(volts, min)


def stop_index(values, threshold):
    """First index after which every |value| stays below threshold."""
    for i in range(len(values)):
        if all(abs(v) < threshold for v in values[i:]):
            return i
    return None


def peak_decel(times_ms, speeds, span=3):
    best = math.nan
    for i in range(span, len(speeds)):
        dt = (times_ms[i] - times_ms[i - span]) / 1000.0
        if dt > 0:
            d = (abs(speeds[i - span]) - abs(speeds[i])) / dt
            best = d if math.isnan(best) else max(best, d)
    return best


def motion(rows, rate_key, travel_key, threshold):
    t = [num(r['t_ms']) for r in rows]
    v = [num(r[rate_key]) for r in rows]
    x = [num(r[travel_key]) for r in rows]
    pitch = [num(r.get('pitch_deg')) for r in rows]
    power = abs(num(rows[0].get('power')))
    drive = [i for i, r in enumerate(rows) if r.get('phase') in ('ACCEL', 'CRUISE')]
    brake = [i for i, r in enumerate(rows) if r.get('phase') == 'BRAKE']
    res = {'peak_rate': peak([abs(s) for s in finite(v)])}
    if drive:
        top = max(abs(v[i]) for i in drive)
        reach = next((i for i in drive if abs(v[i]) >= 0.9 * top), None)
        res['t90_ms'] = t[reach] - t[0] if reach is not None and top > 0 else math.nan
        cruise = [i for i in drive if rows[i].get('phase') == 'CRUISE']
        if cruise and t[cruise[-1]] - t[cruise[0]] >= 300 and power > 0:
            tail = [abs(v[i]) for i in cruise if t[i] >= t[cruise[-1]] - 200]
            res['rate_per_power'] = sum(tail) / len(tail) / power
    if brake:
        b = brake[0]
        start = drive[-1] if drive else b
        res['rate_at_brake'] = abs(v[start])
        idx = stop_index([v[i] for i in brake], threshold)
        if idx is not None:
            s = brake[idx]
            res['stop_ms'] = t[s] - t[b]
            res['stop_travel'] = abs(x[s] - x[start])
        res['peak_decel'] = peak_decel([t[i] for i in [start] + brake], [v[i] for i in [start] + brake])
        base = pitch[start]
        res['brake_pitch_deg'] = peak([abs(pitch[i] - base) for i in brake if not math.isnan(pitch[i])])
    res['current_peak_A'], res['hub_min_V'] = electrical(rows)
    return res


def heading(rows):
    step = num(rows[0].get('step_deg'))
    direction = num(rows[0].get('direction'))
    t = [num(r['t_ms']) for r in rows]
    along = [direction * num(r['turned_deg']) for r in rows]
    res = {'overshoot_deg': max(0.0, max(finite(along)) - step) if finite(along) else math.nan,
           'final_error_deg': step - along[-1],
           'peak_rate': peak([abs(num(r['yaw_rate_dps'])) for r in rows if not math.isnan(num(r['yaw_rate_dps']))])}
    idx = stop_index([a - step for a in along], SETTLE_DEG)
    res['settle_ms'] = t[idx] - t[0] if idx is not None else math.nan
    res['current_peak_A'], res['hub_min_V'] = electrical(rows)
    return res


def summarise(path):
    rows = load(path)
    result = []
    for tr in trials(rows):
        first, last = tr[0], tr[-1]
        kind = first.get('trial')
        item = {'file': Path(path).name, 'trial_id': first.get('trial_id'), 'trial': kind,
                'direction': first.get('direction'), 'brake': first.get('brake'),
                'power': num(first.get('power')), 'step_deg': num(first.get('step_deg')),
                'stop_reason': last.get('stop_reason') or last.get('fault') or 'TRUNCATED',
                'duration_ms': num(last['t_ms']) - num(first['t_ms'])}
        if kind == 'STRAIGHT':
            item.update(motion(tr, 'speed_mps', 'travel_m', STOP_MPS))
        elif kind == 'TURN':
            item.update(motion(tr, 'yaw_rate_dps', 'turned_deg', STOP_DPS))
        elif kind == 'HEADING':
            item.update(heading(tr))
        result.append(item)
    return result


def fmt(value, digits=2):
    if isinstance(value, float):
        return 'nan' if math.isnan(value) else f'{value:.{digits}f}'
    return str(value)


def report(path, items):
    props = properties(path)
    print(f"== {Path(path).name}  gear={props.get('gear_ratio', '?')} accel_per_s={props.get('accel_per_s', '?')} "
          f"brake_per_s={props.get('brake_per_s', '?')} imu_kp={props.get('imu_kp', '?')} imu_kd={props.get('imu_kd', '?')}")
    for it in items:
        head = f"#{it['trial_id']} {it['trial']:<8} dir={it['direction']:>2} {it['stop_reason']:<16}"
        if it['trial'] == 'HEADING':
            print(f"{head} step={fmt(it['step_deg'], 0)}deg overshoot={fmt(it['overshoot_deg'], 1)}deg "
                  f"settle={fmt(it['settle_ms'], 0)}ms final_err={fmt(it['final_error_deg'], 1)}deg "
                  f"peak={fmt(it['peak_rate'], 0)}deg/s I={fmt(it['current_peak_A'])}A Vmin={fmt(it['hub_min_V'])}V")
            continue
        unit = 'm/s' if it['trial'] == 'STRAIGHT' else 'deg/s'
        dist = 'm' if it['trial'] == 'STRAIGHT' else 'deg'
        print(f"{head} brake={it['brake']:<5} power={fmt(it['power'])} t90={fmt(it.get('t90_ms', math.nan), 0)}ms "
              f"per_power={fmt(it.get('rate_per_power', math.nan))}{unit} at_brake={fmt(it.get('rate_at_brake', math.nan))}{unit} "
              f"stop={fmt(it.get('stop_ms', math.nan), 0)}ms/{fmt(it.get('stop_travel', math.nan))}{dist} "
              f"decel_peak={fmt(it.get('peak_decel', math.nan), 1)}{unit}/s pitch={fmt(it.get('brake_pitch_deg', math.nan), 1)}deg "
              f"I={fmt(it['current_peak_A'])}A Vmin={fmt(it['hub_min_V'])}V")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv', nargs='+')
    parser.add_argument('--json', action='store_true', help='print machine-readable results')
    args = parser.parse_args(argv)
    everything = []
    for path in args.csv:
        items = summarise(path)
        everything.extend(items)
        if not args.json:
            report(path, items)
    if args.json:
        clean = [{k: None if isinstance(v, float) and math.isnan(v) else v for k, v in it.items()} for it in everything]
        print(json.dumps(clean, indent=2))


if __name__ == '__main__':
    main()
