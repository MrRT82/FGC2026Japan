import sys
import unittest
import csv
import json
import tempfile
from unittest.mock import patch
import cv2
from types import SimpleNamespace
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from extend_analysis import fit_model, validate, prepare, trials, electrical, analyze, gravity, simulate
from track_extend_video import marker, angle_of, main as video_main


def synthetic(phase=0):
    rows, a, v, dt = [], 35., 0., .02
    for n in range(400):
        t = n * dt
        u = .10 + .12 * np.sin(2 * t + phase) + .04 * np.sin(5 * t + phase)
        rows.append(dict(angle_t_ms=str(t*1000), output_t_ms=str(t*1000),
                         angle_deg=str(a), sent_out=str(u), state='ACTIVE', fault=''))
        for _ in range(20):
            v += dt/20 * (100*u - 3*v - 10)
            a += dt/20 * v
    return rows


class AnalysisTests(unittest.TestCase):
    def test_candidate_simulation_and_extrapolation_rejection(self):
        model = dict(b=100, damping=3, bias=-10, angle_min=0, angle_max=80, input_min=-.25, input_max=.25)
        cfg = dict(max_power=.25, min_angle_deg=1, max_angle_deg=75, friction=0,
                   gravity_cos=0, gravity_constant=.1, gravity_phase_deg=0)
        score = simulate(model, cfg, .02, .005, 35, 5)
        self.assertIsNotNone(score)
        self.assertLess(score['final_error_deg'], 1)
        model['input_max'] = .05
        self.assertIsNone(simulate(model, cfg, .02, .005, 35, 5))

    def test_video_pipeline(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            path = root/'input.mp4'
            w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), 30, (160, 120))
            self.assertTrue(w.isOpened())
            for _ in range(30):
                f = np.zeros((120, 160, 3), dtype=np.uint8)
                cv2.circle(f, (30, 80), 7, (0, 255, 0), -1)
                cv2.circle(f, (110, 40), 7, (255, 0, 0), -1)
                w.write(f)
            w.release()
            cfg = Path(__file__).resolve().parents[1]/'tools'/'video_config.example.json'
            with patch.object(sys, 'argv', ['track', str(path), str(cfg), '--out', str(root/'out')]):
                video_main()
            report = json.loads((root/'out'/'video_report.json').read_text())
            self.assertEqual(report['frames'], 30)
            self.assertEqual(report['valid_fraction'], 1)
            self.assertTrue((root/'out'/'tracked.mp4').exists())

    def test_report_and_gravity_files(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            intervals = []
            for j, angle in enumerate([5, 25, 50, 75]):
                rows = synthetic()[:100]
                for n, r in enumerate(rows):
                    r.update(t_ms=r['angle_t_ms'], angle_deg=str(angle), target_deg=str(angle),
                             goal_deg=str(angle), velocity_dps='0', motor_current_A_seq='0',
                             sent_out=str(.08*np.cos(np.deg2rad(angle+17))+.05))
                path = root / f'{j}.csv'
                with path.open('w', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
                path.with_suffix('.properties').write_text('robot=4\n', encoding='utf-8')
                intervals.append(dict(csv=path.name, start_s=0, end_s=2))
            analyze(SimpleNamespace(csv=path, properties=None, out=root/'report', fit=False))
            self.assertTrue((root/'report'/'signals.png').exists())
            self.assertEqual(json.loads((root/'report'/'report.json').read_text())['summary']['rows'], 100)
            interval_path = root/'intervals.json'
            interval_path.write_text(json.dumps(intervals))
            gravity(SimpleNamespace(intervals=interval_path, out=root/'gravity.json'))
            result = json.loads((root/'gravity.json').read_text())
            self.assertAlmostEqual(result['gravity_cos'], .08)
            self.assertAlmostEqual(result['gravity_phase_deg'], 17)

    def test_model_recovery_and_independent_replay(self):
        model = fit_model(synthetic())
        self.assertAlmostEqual(model['b'], 100, delta=10)
        self.assertAlmostEqual(model['damping'], 3, delta=.4)
        metrics, _ = validate(model, synthetic(.7))
        self.assertLess(metrics['rmse_deg'], 1)

    def test_static_rejected(self):
        rows = synthetic()
        for r in rows: r['angle_deg'] = '35'
        with self.assertRaisesRegex(ValueError, 'excitation'): fit_model(rows)

    def test_fault_and_gap_split(self):
        rows = synthetic()
        rows[100]['state'] = 'FAULT'
        self.assertEqual([len(r) for r in trials(rows)], [100, 299])
        rows[100]['state'] = 'ACTIVE'
        del rows[100:110]
        self.assertEqual([len(r) for r in trials(rows)], [100, 290])

    def test_command_does_not_apply_before_timestamp(self):
        rows = synthetic()
        for r in rows: r['output_t_ms'] = str(float(r['angle_t_ms']) + 10)
        t, _, u = prepare(rows)
        self.assertAlmostEqual(t[0], .01)
        self.assertAlmostEqual(u[0], float(rows[0]['sent_out']))

    def test_cached_current_is_not_duplicate(self):
        row = dict(t_ms='100', current_seq='1', current_age_ms='20', current='2', current_ok='1')
        result = electrical([row, dict(row, t_ms='120', current_age_ms='40')], 'current')
        np.testing.assert_allclose(result, [[.08, 2]])

    def test_markers_ambiguity_and_angles(self):
        hsv = np.zeros((100, 100, 3), dtype=np.uint8)
        hsv[10:30, 10:30] = [60, 200, 200]
        self.assertIsNotNone(marker(hsv, [40, 100, 80], [85, 255, 255], 30))
        hsv[60:80, 60:80] = [60, 200, 200]
        self.assertIsNone(marker(hsv, [40, 100, 80], [85, 255, 255], 30))
        self.assertEqual(angle_of((50, 50), (50, 10), 0, 1), 90)


if __name__ == '__main__': unittest.main()
