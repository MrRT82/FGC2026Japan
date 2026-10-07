"""Synthetic Drive_Tune_R4 CSVs; checks the per-trial metrics, not robot behaviour."""
import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import analyze_drive_tune as tool  # noqa: E402

COLUMNS = ['t_ms', 'state', 'fault', 'trial_id', 'trial', 'phase', 'event', 'stop_reason', 'direction', 'brake',
           'power', 'step_deg', 'speed_mps', 'travel_m', 'yaw_rate_dps', 'turned_deg', 'pitch_deg',
           'left_current_A', 'left_current_A_ok', 'left_current_A_seq',
           'hub_0_voltage_V', 'hub_0_voltage_V_ok', 'hub_0_voltage_V_seq']


def row(t, state='RUNNING', phase='', event='', trial='STRAIGHT', speed=0.0, travel=0.0, rate=0.0, turned=0.0,
        pitch=0.0, amps=1.0, seq=0, volts=12.5, stop=''):
    return {'t_ms': t, 'state': state, 'fault': '', 'trial_id': 1, 'trial': trial, 'phase': phase, 'event': event,
            'stop_reason': stop, 'direction': 1, 'brake': 'MATCH', 'power': 0.5, 'step_deg': 45,
            'speed_mps': speed, 'travel_m': travel, 'yaw_rate_dps': rate, 'turned_deg': turned, 'pitch_deg': pitch,
            'left_current_A': amps, 'left_current_A_ok': 1, 'left_current_A_seq': seq,
            'hub_0_voltage_V': volts, 'hub_0_voltage_V_ok': 1, 'hub_0_voltage_V_seq': seq}


def write(rows, truncated=False):
    path = Path(tempfile.mkdtemp()) / 'drive.csv'
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
        if truncated:
            f.write('9999,RUNNING')
    return path


def straight_rows():
    rows = [row(0, phase='ACCEL', event='START')]
    t, x = 0, 0.0
    for k in range(1, 31):  # 5 m/s^2 to 1 m/s, then cruise until 600 ms
        t += 20
        v = min(1.0, k * 0.1)
        x += v * 0.02
        rows.append(row(t, phase='ACCEL' if k < 10 else 'CRUISE', speed=v, travel=x, seq=t // 20))
    for k in range(25, -1, -1):  # sensors precede the output, so the first BRAKE row still shows 1 m/s
        t += 20
        v = k * 0.04
        x += v * 0.02
        rows.append(row(t, phase='BRAKE', speed=v, travel=x, pitch=3.0 if v > 0.5 else 1.0,
                        amps=9.0 if v > 0.9 else 2.0, volts=11.2 if v > 0.9 else 12.4, seq=t // 20))
    for _ in range(10):
        t += 20
        rows.append(row(t, phase='BRAKE', travel=x, seq=t // 20))
    rows.append(row(t + 20, state='IDLE', phase='DONE', event='END', travel=x, stop='STOPPED'))
    rows.append(row(t + 40, state='IDLE', phase='DONE', travel=x))
    return rows


class DriveAnalysisTest(unittest.TestCase):
    def test_straight_accel_and_brake(self):
        (item,) = tool.summarise(write(straight_rows(), truncated=True))
        self.assertEqual(item['stop_reason'], 'STOPPED')
        self.assertAlmostEqual(item['t90_ms'], 180, delta=1)
        self.assertAlmostEqual(item['rate_per_power'], 2.0, places=6)
        self.assertAlmostEqual(item['rate_at_brake'], 1.0, places=6)
        self.assertAlmostEqual(item['stop_ms'], 500, places=6)
        self.assertAlmostEqual(item['stop_travel'], 0.26, places=6)  # v^2/2a = 0.25 plus 20 ms sampling
        self.assertAlmostEqual(item['peak_decel'], 2.0, delta=0.05)
        self.assertEqual(item['brake_pitch_deg'], 3.0)
        self.assertEqual(item['current_peak_A'], 9.0)
        self.assertEqual(item['hub_min_V'], 11.2)

    def test_heading_overshoot_and_settle(self):
        along = [0, 10, 25, 40, 50, 49, 47, 46, 45, 45, 45]
        rows = [row(i * 100, trial='HEADING', phase='HOLD', event='START' if i == 0 else '', turned=a, rate=50)
                for i, a in enumerate(along)]
        rows.append(row(1100, state='IDLE', trial='HEADING', phase='DONE', event='END', turned=45, stop='TIME'))
        (item,) = tool.summarise(write(rows))
        self.assertAlmostEqual(item['overshoot_deg'], 5)
        self.assertEqual(item['settle_ms'], 600)
        self.assertEqual(item['final_error_deg'], 0)

    def test_fault_row_closes_trial(self):
        rows = straight_rows()[:5]
        rows.append(row(100, state='FAULT', phase='CRUISE', stop='LOOP_DELAY'))
        (item,) = tool.summarise(write(rows))
        self.assertEqual(item['stop_reason'], 'LOOP_DELAY')
        self.assertNotIn('stop_ms', item)


if __name__ == '__main__':
    unittest.main()
