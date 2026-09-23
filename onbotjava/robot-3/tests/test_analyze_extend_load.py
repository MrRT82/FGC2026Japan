import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('load_analysis',
    Path(__file__).resolve().parents[1] / 'tools/analyze_extend_load.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def sample(seq, **changes):
    values = dict(seq=seq, initial_balls=3, t_ms=seq * 100, current_A=2,
                  current_ok=1, read_ms=2, angle_deg=35, velocity_dps=10,
                  angle_age_ms=20, target_deg=40, goal_deg=76, requested_out=.3,
                  sent_out=.3, ff=.1, pid=.2, i=0, control_dt_ms=25,
                  raw_dt_ms=25, stuck=0, reverse=0, stall=0, pote_fault=0,
                  **{'yield': 0}, voltage_V=12, voltage_age_ms=100, period_ms=25)
    values.update(changes)
    return {'load_' + k: v for k, v in values.items()}


class LoadAnalysisTest(unittest.TestCase):
    def analyze(self, rows, **kwargs):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'load.csv'
            with path.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(sample(1)))
                writer.writeheader()
                writer.writerows(rows)
            return module.analyze(path, **kwargs)

    def test_cached_rows_not_counted_and_runs_grouped(self):
        groups, rejected = self.analyze([sample(1), sample(1), sample(2), sample(2),
                                        sample(3, current_A=4)])
        self.assertEqual(len(groups[(3, 30, 'positive', 10)]), 2)
        self.assertEqual(rejected['repeated_cached_row'], 2)

    def test_failures_delays_and_protection_are_excluded(self):
        groups, rejected = self.analyze([sample(1), sample(2, current_A='NaN', current_ok=0),
                                        sample(3), sample(4, angle_age_ms=200),
                                        sample(5, reverse=1), sample(6, raw_dt_ms=200),
                                        sample(7, voltage_age_ms=1000), sample(8)])
        self.assertEqual(sum(map(len, groups.values())), 1)
        self.assertEqual(rejected['nonfinite_or_failed'], 1)
        self.assertEqual(rejected['stale_or_delayed_control'], 2)
        self.assertEqual(rejected['protection_or_reverse'], 1)
        self.assertEqual(rejected['invalid_or_stale_voltage'], 1)

    def test_acceleration_direction_and_time_selection(self):
        groups, rejected = self.analyze([sample(1, velocity_dps=-10), sample(2, velocity_dps=-10),
                                        sample(3, velocity_dps=20), sample(4, velocity_dps=20)],
                                       end_ms=350)
        self.assertEqual(len(groups[(3, 30, 'negative', 10)]), 1)
        self.assertEqual(rejected['acceleration_or_gap'], 2)
        self.assertEqual(rejected['outside_time_range'], 1)


if __name__ == '__main__':
    unittest.main()
