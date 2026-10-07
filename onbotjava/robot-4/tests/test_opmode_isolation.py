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

    def test_shoot_routes_leave_extension_then_raise_slowly(self):
        # 2026-10-08: 射撃開始時の中央角への移動は廃止。送り開始・B/X排出から1秒後に低速で上げ、LBは全力で下ろす。
        controller = (SOURCE / 'ExtendController.java').read_text(encoding='utf-8')
        self.assertEqual(len(re.findall(
            r'\bdouble\s+pote_Middle\s*=\s*[-+]?(?:\d+(?:\.\d*)?|\.\d+)\s*;', controller)), 1)
        self.assertIn('TARGET_SLEW_SLOW_DPS = TARGET_SLEW_DPS / 3.0', controller)
        extension_lines = {}
        for name in ('MainClass_Blue.java', 'MainClass_Red.java'):
            text = (SOURCE / name).read_text(encoding='utf-8')
            with self.subTest(path=name):
                self.assertNotIn('extendController.pote_Middle', text,
                                 'shooting must not move the extension to the middle angle')
                self.assertEqual(text.count('commandExtendSlow(extendController.pote_Up);'), 1)
                self.assertEqual(text.count('commandExtendDrop(extendController.pote_Down);'), 1)
                self.assertIn('EXTEND_RAISE_DELAY_MS = 1000', text)
                self.assertEqual(text.count('resetExtendRaise();'), 3, 'RT, A and session end reset the raise')
            extension_lines[name] = [line.strip() for line in text.splitlines()
                                     if re.search(r'commandExtend|extendRaise|EXTEND_RAISE|raiseRequested', line)]
        self.assertEqual(extension_lines['MainClass_Blue.java'], extension_lines['MainClass_Red.java'],
                         'Blue and Red must share the same extension sequence')

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
