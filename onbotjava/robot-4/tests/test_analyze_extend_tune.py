import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('tune_analysis', ROOT / 'tools/analyze_extend_tune.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TuneAnalysisTest(unittest.TestCase):
    def test_unique_samples_invalid_and_partial_tail(self):
        fields = ['t_ms', 'angle_t_ms', 'output_t_ms', 'angle_deg', 'target_deg', 'state', 'fault',
                  'motor_current_A', 'motor_current_A_seq', 'motor_current_A_ok',
                  'motor_current_A_age_ms', 'motor_current_A_read_ms', 'period_ms']
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'test.csv'
            with path.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerow([100, 90, 101, 39, 40, 'ACTIVE', '', 2, 1, 1, 10, 3, 20])
                writer.writerow([120, 110, 121, 39, 40, 'ACTIVE', '', 2, 1, 1, 30, 3, 20])
                writer.writerow([140, 130, 141, 40, 40, 'FAULT', 'ELECTRICAL_STALE_OR_INVALID', 'NaN', 2, 0, 0, 5, 20])
                stream.write('160,155,')
            result = module.summarize(path)
            current = result['electrical']['motor_current_A']
            self.assertEqual(current['samples'], 1)
            self.assertEqual(current['invalid'], 1)
            self.assertEqual(current['max'], 2)
            self.assertEqual(current['max_at_ms'], 90)
            self.assertEqual(result['active_tracking_rmse_deg'], 1)
            self.assertEqual(result['skipped_incomplete_rows'], 1)

    def test_wrong_log_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'old.csv'
            path.write_text('period_ms\n20\n')
            with self.assertRaises(ValueError):
                module.summarize(path)

    def test_power_signals_remain_separate(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'power.csv'
            path.write_text('t_ms,seq,signal,value,ok,read_ms\n'
                            '10,1,hub_0_V,12,1,2\n20,2,hub_1_V,10,1,3\n'
                            '30,3,extend_A,NaN,0,4\n40,4,extend_A,2,1,4\n50,')
            result = module.summarize(path)
            self.assertEqual(result['electrical']['hub_0_V']['min'], 12)
            self.assertEqual(result['electrical']['hub_1_V']['min'], 10)
            self.assertEqual(result['electrical']['extend_A']['invalid'], 1)
            self.assertEqual(result['skipped_incomplete_rows'], 1)

    def test_robot4_timing_mechanism_settings_match_normal(self):
        import re
        source = ROOT / 'src/org/firstinspires/ftc/teamcode'
        normal = (source / 'ExtendController.java').read_text(encoding='utf-8')
        timing = (source / 'timing/ExtendController.java').read_text(encoding='utf-8')
        keys = ['pote_offset', 'pote_Middle', 'pote_Up', 'pote_Down', 'TARGET_MIN_DEG',
                'TARGET_MAX_DEG', 'REVERSE_TIME_MS', 'POTE_ANGLE_MIN', 'POTE_ANGLE_MAX',
                'kp', 'ki', 'kd', 'KG_AMP', 'KG_CONST', 'KS_FF', 'gravity_offset_angle']
        for key in keys:
            pattern = rf'\b{key}\s*=\s*([^;]+);'
            self.assertEqual(re.search(pattern, normal)[1], re.search(pattern, timing)[1], key)
        for name in ['Feed_turn_right', 'Feed_turn_left']:
            pattern = rf'void {name}\(\).*?feedOut.set\((.*?)\);'
            a = (source / 'FeedController.java').read_text(encoding='utf-8')
            b = (source / 'timing/FeedController.java').read_text(encoding='utf-8')
            self.assertEqual(re.search(pattern, a, re.S)[1], re.search(pattern, b, re.S)[1])


if __name__ == '__main__':
    unittest.main()
