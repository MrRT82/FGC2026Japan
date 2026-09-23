import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "analyze_timing", Path(__file__).resolve().parents[1] / "tools/analyze_timing.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AnalyzeTimingTest(unittest.TestCase):
    def test_weighted_aggregation_and_incomplete_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "synthetic.csv"
            path.write_text(
                "tm_imu_ypr_n,tm_imu_ypr_total_us,tm_imu_ypr_max_us,tm_imu_ypr_errors\n"
                "1,1000,1000,0\n3,9000,5000,1\n2,\n", encoding="utf-8")
            totals, rows, skipped = module.summarize(path)
            self.assertEqual(totals["imu_ypr"], [4, 10000, 5000, 1])
            self.assertEqual((rows, skipped), (2, 1))

    def test_old_csv_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.csv"
            path.write_text("period_ms\n25\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                module.summarize(path)


if __name__ == "__main__":
    unittest.main()
