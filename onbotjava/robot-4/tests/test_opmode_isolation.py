"""Keep production/diagnostic packages separate and reject removed sensor dependencies."""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
SOURCE = ROOT / 'src/org/firstinspires/ftc/teamcode'


class OpModeIsolationTest(unittest.TestCase):
    def test_normal_dependencies_are_isolated(self):
        files = list(SOURCE.glob('*.java'))
        self.assertEqual(len(files), 22)
        for path in files:
            with self.subTest(path=path.name):
                text = path.read_text(encoding='utf-8')
                self.assertIn('package org.firstinspires.ftc.teamcode;', text)
                self.assertNotRegex(text, r'org\.firstinspires\.ftc\.teamcode\.(timing|tuning)\.')

    def test_removed_sensor_has_no_source_dependencies(self):
        self.assertFalse((SOURCE / 'Distance.java').exists())
        self.assertFalse((SOURCE / 'timing/Distance.java').exists())
        for path in SOURCE.rglob('*.java'):
            with self.subTest(path=str(path.relative_to(SOURCE))):
                self.assertNotRegex(path.read_text(encoding='utf-8'),
                                    r'\bDistanceSensor\b|\bDistanceUnit\b|\bdistanceSensor\b|'
                                    r'\bDistance\b|driveWithDistanceAndImu|\bdst_\w+|tm_distance')

    def test_exactly_three_main_registration_names(self):
        names = []
        for path in SOURCE.rglob('MainClass*.java'):
            text = path.read_text(encoding='utf-8')
            names.extend(re.findall(r'@TeleOp\(name="([^"]+)"', text))
        self.assertCountEqual(names, ['MainClass_Blue', 'MainClass_Red', 'MainClass_Blue_Timing'])

    def test_timing_dependencies_are_isolated(self):
        files = list((SOURCE / 'timing').glob('*.java'))
        self.assertEqual(len(files), 21)
        for path in files:
            text = path.read_text(encoding='utf-8')
            with self.subTest(path=path.name):
                self.assertIn('package org.firstinspires.ftc.teamcode.timing;', text)
                self.assertNotRegex(text, r'import\s+org\.firstinspires\.ftc\.teamcode\.(?!timing\.|tuning\.TuneCsvWriter;)')
        self.assertFalse((SOURCE / 'SensorReads.java').exists())
        self.assertFalse((SOURCE / 'SensorTiming.java').exists())
        timing_logger = (SOURCE / 'timing/RobotLogger.java').read_text(encoding='utf-8')
        self.assertIn('"robot_blue_timing_"', timing_logger)


if __name__ == '__main__':
    unittest.main()
