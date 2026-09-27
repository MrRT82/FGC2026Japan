"""Inspect robot-4 tune CSVs without inventing gains from incomplete experiments.

Standard-library only. Electrical extrema count each sample sequence once.
An interrupted/truncated CSV is accepted, but reported as incomplete.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def summarize(path):
    signals, seen, rows, skipped, faults, errors, periods = {}, {}, 0, 0, set(), [], []
    last = None
    with open(path, encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if {'signal', 'value', 'ok', 't_ms', 'read_ms', 'seq'}.issubset(fields):
            return summarize_power(path)
        required = {'t_ms', 'angle_t_ms', 'output_t_ms', 'angle_deg', 'target_deg', 'state', 'motor_current_A_seq'}
        if not required.issubset(fields):
            raise ValueError('Not an Extend_Tune_R4 CSV')
        names = [name[:-4] for name in fields if name.endswith('_seq')]
        for name in names:
            signals[name] = {'samples': 0, 'invalid': 0, 'min': None, 'max': None,
                             'max_at_ms': None, 'min_at_ms': None, 'max_read_ms': 0}
            seen[name] = set()
        for row in reader:
            t = finite(row.get('t_ms'))
            if None in row or any(value is None for value in row.values()) or t is None:
                skipped += 1
                continue
            rows += 1
            last = row
            if row.get('fault'):
                faults.add(row['fault'])
            period = finite(row.get('period_ms'))
            if period is not None:
                periods.append(period)
            for name in names:
                seq = finite(row.get(name + '_seq'))
                if seq is None or seq <= 0 or seq in seen[name]:
                    continue
                seen[name].add(seq)
                result = signals[name]
                duration = finite(row.get(name + '_read_ms'))
                if duration is not None:
                    result['max_read_ms'] = max(result['max_read_ms'], duration)
                value, age = finite(row.get(name)), finite(row.get(name + '_age_ms'))
                if row.get(name + '_ok') != '1' or value is None or age is None or age < 0:
                    result['invalid'] += 1
                    continue
                result['samples'] += 1
                if result['min'] is None or value < result['min']:
                    result['min'], result['min_at_ms'] = value, t - age
                if result['max'] is None or value > result['max']:
                    result['max'], result['max_at_ms'] = value, t - age
            angle, target = finite(row.get('angle_deg')), finite(row.get('target_deg'))
            if row.get('state') == 'ACTIVE' and angle is not None and target is not None:
                errors.append(target - angle)
    return {
        'file': str(path), 'rows': rows, 'skipped_incomplete_rows': skipped,
        'electrical': signals, 'faults': sorted(faults),
        'last_state': last.get('state') if last else None,
        'last_t_ms': finite(last.get('t_ms')) if last else None,
        'max_loop_period_ms': max(periods) if periods else None,
        'active_tracking_rmse_deg': math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None,
        'active_max_abs_error_deg': max(map(abs, errors)) if errors else None,
        'note': 'Sampled extrema only. Missing final rows or peaks do not exclude power loss. No gains inferred.'
    }


def summarize_power(path):
    signals, rows, skipped, last_t = {}, 0, 0, None
    with open(path, encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            t = finite(row.get('t_ms'))
            if None in row or any(v is None for v in row.values()) or t is None:
                skipped += 1
                continue
            rows += 1
            last_t = t
            r = signals.setdefault(row['signal'], {'samples': 0, 'invalid': 0, 'min': None,
                'max': None, 'min_at_ms': None, 'max_at_ms': None, 'max_read_ms': 0})
            duration, value = finite(row.get('read_ms')), finite(row.get('value'))
            if duration is not None:
                r['max_read_ms'] = max(r['max_read_ms'], duration)
            if row['ok'] != '1' or value is None:
                r['invalid'] += 1
                continue
            r['samples'] += 1
            if r['min'] is None or value < r['min']:
                r['min'], r['min_at_ms'] = value, t
            if r['max'] is None or value > r['max']:
                r['max'], r['max_at_ms'] = value, t
    return {'file': str(path), 'format': 'robot4_power', 'rows': rows,
            'skipped_incomplete_rows': skipped, 'last_t_ms': last_t, 'electrical': signals,
            'note': 'Signals are non-simultaneous; no battery-current sum or shutdown cause inferred.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, nargs='+')
    args = parser.parse_args()
    for path in args.csv:
        print(json.dumps(summarize(path), ensure_ascii=False, indent=2, allow_nan=False))
