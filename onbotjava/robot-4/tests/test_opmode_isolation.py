"""Ensure the ordinary robot programs retain their pre-instrumentation code."""
import re
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
SOURCE = ROOT / 'src/org/firstinspires/ftc/teamcode'
BASELINE = 'd482df983993daa352928fc54437c8ece35bb5a3'


class OpModeIsolationTest(unittest.TestCase):
    def test_all_normal_sources_are_unchanged(self):
        paths = subprocess.check_output(
            ['git', 'ls-tree', '-r', '--name-only', BASELINE, 'onbotjava/robot-4/src'],
            cwd=REPO, text=True).splitlines()
        self.assertEqual(len(paths), 22)
        self.assertEqual(len(list(SOURCE.glob('*.java'))), 22)
        for path in paths:
            with self.subTest(path=path):
                original = subprocess.check_output(['git', 'show', f'{BASELINE}:{path}'], cwd=REPO)
                # Git checkout may convert LF to CRLF on Windows; code and all text must match.
                actual = (REPO / path).read_bytes().replace(b'\r\n', b'\n')
                self.assertEqual(actual, original.replace(b'\r\n', b'\n'))

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
                self.assertNotRegex(text, r'import\s+org\.firstinspires\.ftc\.teamcode\.(?!timing\.)')
        self.assertFalse((SOURCE / 'SensorReads.java').exists())
        self.assertFalse((SOURCE / 'SensorTiming.java').exists())
        timing_logger = (SOURCE / 'timing/RobotLogger.java').read_text(encoding='utf-8')
        self.assertIn('"robot_blue_timing_"', timing_logger)


if __name__ == '__main__':
    unittest.main()
