"""OPTIONAL diagnostic: external motion versus the angle sensor, if discrepant.

Not part of model fitting/gain tuning; CAD calibration + CSV is the main flow.
Track two colored markers in a fixed, side-on, constant-frame-rate video.

Config JSON: pivot_hsv_low/high, tip_hsv_low/high (OpenCV HSV ranges),
min_area_px, min_length_px, zero_deg, direction (+1/-1), log_offset_s.
log_time = video_time + log_offset_s. No automatic synchronization.
"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def marker(hsv, low, high, min_area):
    mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    good = [c for c in contours if cv2.contourArea(c) >= min_area]
    # Ambiguous frames become missing, rather than silently switching identity.
    if len(good) != 1:
        return None
    m = cv2.moments(good[0])
    return None if not m['m00'] else (m['m10'] / m['m00'], m['m01'] / m['m00'])


def angle_of(pivot, tip, zero, direction):
    raw = np.degrees(np.arctan2(pivot[1] - tip[1], tip[0] - pivot[0]))
    return float((direction * (raw - zero) + 180) % 360 - 180)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('video', type=Path); p.add_argument('config', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--log', type=Path, help='Optional Tune CSV for timestamp-aligned angle overlay')
    args = p.parse_args()
    cfg = json.loads(args.config.read_text(encoding='utf-8-sig'))
    if cfg['direction'] not in (-1, 1): raise ValueError('direction must be +/-1')
    for prefix in ('pivot', 'tip'):
        lo, hi = np.array(cfg[prefix + '_hsv_low']), np.array(cfg[prefix + '_hsv_high'])
        if lo.shape != (3,) or hi.shape != (3,) or np.any(lo < 0) or np.any(hi > [179, 255, 255]) or np.any(lo > hi):
            raise ValueError('Invalid OpenCV HSV range')
    cap = cv2.VideoCapture(str(args.video))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not cap.isOpened() or not np.isfinite(fps) or fps <= 0:
        raise ValueError('Unreadable video or invalid FPS')
    args.out.mkdir(parents=True, exist_ok=True)
    writer, data, n = None, [], 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok: break
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(str(args.out / 'tracked.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                if not writer.isOpened(): raise ValueError('Cannot create annotated video')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            points = [marker(hsv, cfg[k + '_hsv_low'], cfg[k + '_hsv_high'], cfg['min_area_px']) for k in ('pivot', 'tip')]
            angle = np.nan
            if all(x is not None for x in points) and np.linalg.norm(np.subtract(*points)) >= cfg['min_length_px']:
                angle = angle_of(*points, cfg['zero_deg'], cfg['direction'])
                cv2.line(frame, tuple(map(round, points[0])), tuple(map(round, points[1])), (255, 255, 255), 2)
            video_t = n / fps
            data.append([n, video_t, video_t + cfg['log_offset_s'], angle, int(np.isfinite(angle))])
            cv2.putText(frame, f'{video_t:.3f}s angle={angle:.2f}', (12, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
            writer.write(frame); n += 1
    finally:
        cap.release()
        if writer is not None: writer.release()
    if not data: raise ValueError('No frames decoded')
    with (args.out / 'video_angles.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f); w.writerow(['frame', 'video_s', 'log_s', 'angle_deg', 'valid']); w.writerows(data)
    z = np.array(data)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(z[:, 2], z[:, 3], label='video (projected angle)')
    if args.log:
        from extend_analysis import load, col
        rows, _ = load(args.log)
        ax.plot(col(rows, 'angle_t_ms') / 1000, col(rows, 'angle_deg'), label='sensor')
    ax.set(xlabel='log seconds (manual offset)', ylabel='degrees'); ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(args.out / 'video_overlay.png'); plt.close(fig)
    (args.out / 'video_report.json').write_text(json.dumps(dict(frames=n, fps=fps, valid_fraction=float(np.mean(z[:, 4])), config=cfg,
        note='Frame-index timing assumes constant FPS. Perspective, lens distortion and synchronization error not corrected. Missing/ambiguous markers produce NaN.'), indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
