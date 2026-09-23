"""Compare load samples by run, initial ball count, angle and signed speed (no count estimator)."""
import argparse
from collections import Counter, defaultdict
import csv
import math
from pathlib import Path
import statistics
import sys


def analyze(path, max_accel=50.0, start_ms=0.0, end_ms=float('inf')):
    groups = defaultdict(list)
    rejected = Counter()
    seen = set()
    previous = None
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'load_seq', 'load_initial_balls', 'load_current_A', 'load_raw_dt_ms'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('No load_v3 columns: install the load measurement build.')
        for row in reader:
            try:
                if None in row or any(v is None for v in row.values()):
                    raise ValueError('Incomplete CSV row')
                sample = {k[5:]: float(v) for k, v in row.items() if k.startswith('load_')}
                seq = sample['seq']
                if not math.isfinite(seq) or seq < 1:
                    raise ValueError('Invalid sample id')
                if seq in seen:
                    rejected['repeated_cached_row'] += 1
                    continue
                seen.add(seq)
                if not all(math.isfinite(v) for v in sample.values()):
                    previous = None
                    rejected['nonfinite_or_failed'] += 1
                    continue
                t = sample['t_ms']
                v = sample['velocity_dps']
                accel = None
                if previous is not None:
                    dt = (t - previous[0]) / 1000
                    if 0 < dt <= 0.5:
                        accel = (v - previous[1]) / dt
                previous = (t, v)
                if not start_ms <= t <= end_ms:
                    rejected['outside_time_range'] += 1
                    continue
                if sample['current_ok'] != 1 or sample['current_A'] < 0:
                    rejected['invalid_current'] += 1
                    continue
                if any(sample[k] != 0 for k in ('stuck', 'reverse', 'stall', 'pote_fault', 'yield')):
                    rejected['protection_or_reverse'] += 1
                    continue
                if not (0 <= sample['angle_age_ms'] <= 100 and 0 <= sample['read_ms'] <= 50
                        and 2 <= sample['raw_dt_ms'] <= 100):
                    rejected['stale_or_delayed_control'] += 1
                    continue
                if not (sample['voltage_V'] > 0 and 0 <= sample['voltage_age_ms'] <= 500):
                    rejected['invalid_or_stale_voltage'] += 1
                    continue
                if accel is None or abs(accel) > max_accel:
                    rejected['acceleration_or_gap'] += 1
                    continue
                # Signed speed preserves motion direction; hold is separated from slow motion.
                direction = 'hold' if abs(v) <= 2 else ('positive' if v > 0 else 'negative')
                speed_bin = 0 if direction == 'hold' else math.floor(abs(v) / 5) * 5
                key = (int(sample['initial_balls']), math.floor(sample['angle_deg'] / 10) * 10,
                       direction, speed_bin)
                sample['accel_dps2'] = accel
                groups[key].append(sample)
            except (TypeError, ValueError, KeyError, OverflowError):
                previous = None
                rejected['malformed'] += 1
    return groups, rejected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_paths', type=Path, nargs='+')
    parser.add_argument('--max-accel', type=float, default=50)
    parser.add_argument('--from-ms', type=float, default=0)
    parser.add_argument('--to-ms', type=float, default=float('inf'))
    args = parser.parse_args()
    if not math.isfinite(args.max_accel) or args.max_accel < 0 or args.to_ms < args.from_ms:
        parser.error('Invalid acceleration limit or time range')
    writer = csv.writer(sys.stdout)
    writer.writerow(['run', 'initial_balls', 'angle_bin_deg', 'direction', 'speed_bin_dps',
                     'n', 'current_mean_A', 'current_sd_A', 'current_min_A', 'current_max_A',
                     'voltage_mean_V', 'error_mean_deg', 'sent_out_mean', 'saturated_pct',
                     'period_mean_ms', 'read_mean_ms'])
    for path in args.csv_paths:
        try:
            groups, rejected = analyze(path, args.max_accel, args.from_ms, args.to_ms)
        except ValueError as error:
            parser.error(str(error))
        print(f'{path.name}: excluded={dict(rejected)}', file=sys.stderr)
        for key, samples in sorted(groups.items()):
            currents = [s['current_A'] for s in samples]
            mean = lambda name: statistics.mean(s[name] for s in samples)
            writer.writerow([path.name, *key, len(samples), statistics.mean(currents),
                             statistics.stdev(currents) if len(currents) > 1 else '',
                             min(currents), max(currents), mean('voltage_V'),
                             statistics.mean(s['target_deg'] - s['angle_deg'] for s in samples),
                             mean('sent_out'), 100 * sum(abs(s['requested_out']) >= .99
                                                        for s in samples) / len(samples),
                             mean('period_ms'), mean('read_ms')])
    print('Bins are exploratory, not a ball-count calibration. Compare repeated runs at similar '
          'voltage, angle and speed; initial_balls is invalid after adding/removing balls. '
          'Acceleration uses differences of filtered velocity; thresholds are adjustable.', file=sys.stderr)


if __name__ == '__main__':
    main()
