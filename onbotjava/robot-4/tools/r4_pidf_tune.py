"""Robot 4 extension PIDF tuning, following the FGC_PIDF_Tuner flow (identify -> simulate the Java
law -> grid search with robustness), with velocity feedforward (kV) added.

Robot 4 only: its gear ratio differs from the mechanism FGC_PIDF_Tuner was used on, so nothing is
copied from that tuner's plant; every plant value is identified from robot 4 CSV logs.

Units follow ExtendTuneAutoCore: angle in sensor degrees (V/Vmax*135), speed deg/s, output duty.

    py -3.13 tools/r4_pidf_tune.py identify tuning-data/2026-09-29 --out r4_plant.json
    py -3.13 tools/r4_pidf_tune.py replay r4_plant.json tuning-data/2026-09-29
    py -3.13 tools/r4_pidf_tune.py search r4_plant.json --out r4_gains.json
"""
import argparse, glob, itertools, json, math, os, sys
import numpy as np
import pandas as pd

PHASE_DEG = 17.0          # gravity_phase_deg of the robot 4 config; a free phase is ill-posed over 25-71 deg
DT = 0.020
AUTO_FILES = ('07_', '08_', '09_', '10_')
# Run 11 is stick-slip chatter (+/-134 deg/s): a linear fit on it collapses kV to 0, so it is
# used only to check the closed-loop replay.
REPLAY_FILES = AUTO_FILES + ('11_',)
# Handoff: 06 is the accepted negative breakaway, 01 the positive reference; 02-05 are
# reversed/unsustained angle responses and 04 released the deadman.
BREAKAWAY_FILES = ('01_', '06_')


def load(path):
    d = pd.read_csv(path, low_memory=False)
    d['t'] = d.t_ms / 1000.0
    return d


# ---------------------------------------------------------------- identification
def moving_samples(d):
    a = d[(d.state == 'ACTIVE')].copy()
    if len(a) < 20:
        return None
    v = a.velocity_dps.to_numpy()
    t = a.t.to_numpy()
    acc = np.gradient(v, t)
    acc = pd.Series(acc).rolling(5, center=True, min_periods=1).mean().to_numpy()
    a['acc'] = acc
    # Output is applied after the angle read, so pair each output with the next loop's motion.
    a['u'] = a.sent_out.shift(1)
    a = a.dropna(subset=['u'])
    return a[(a.velocity_dps.abs() > 4) & (a.u.abs() < 0.999)]


def fit_dynamic(frames):
    """duty = A cos(q+phase) + C + Fc+ [v>0] - Fc- [v<0] + kV v + kA a (least squares)."""
    m = pd.concat(frames)
    q, v, a, u = (m[k].to_numpy() for k in ('angle_deg', 'velocity_dps', 'acc', 'u'))
    X = np.column_stack([np.cos(np.radians(q + PHASE_DEG)), np.ones_like(q),
                         (v > 0).astype(float), -(v < 0).astype(float), v, a])
    coef, *_ = np.linalg.lstsq(X, u, rcond=None)
    resid = u - X @ coef
    r2 = 1 - resid.var() / u.var()
    names = ['gravity_cos', 'gravity_constant', 'fc_pos', 'fc_neg', 'kv', 'ka']
    out = dict(zip(names, map(float, coef)))
    out.update(samples=int(len(u)), rms=float(np.sqrt(np.mean(resid ** 2))), r2=float(r2),
               up_samples=int((v > 0).sum()), down_samples=int((v < 0).sum()))
    return out


def breakaway_events(d, grav):
    """Motor duty at the end of an Extend_Tune_R4 BREAKAWAY ramp, relative to the gravity model.

    The direction is the motor effort direction, not the angle direction: the handoff found the
    angle sign near breakaway unreliable (backlash/compliance), so only the effort is used.
    """
    a = d[(d.state == 'ACTIVE') & (d.phase == 'RAMP')]
    if a.empty or not str(d.trial_mode.iloc[0]).startswith('BREAKAWAY'):
        return []
    last = a.iloc[-1]
    live = a[a.sent_out.abs() > 1e-3]
    volts = float(np.median(live.command_V / live.sent_out))   # Hub voltage the ramp was normalised by
    duty = float(last.command_V / volts)
    direction = 1 if last.command_V > last.baseline_V else -1
    g = grav['gravity_cos'] * math.cos(math.radians(last.angle_deg + PHASE_DEG)) + grav['gravity_constant']
    return [dict(angle=float(last.angle_deg), duty=duty, hub_V=volts, gravity=g,
                 net=float(duty - g), direction=direction, hold_duty=float(last.baseline_V / volts))]


def identify(folder):
    files = sorted(glob.glob(os.path.join(folder, '*.csv')))
    auto = [f for f in files if os.path.basename(f).startswith(AUTO_FILES)]
    brk = [f for f in files if os.path.basename(f).startswith(BREAKAWAY_FILES)]
    frames = [s for s in (moving_samples(load(f)) for f in auto) if s is not None]
    dyn = fit_dynamic(frames)
    events = []
    for f in brk:
        for e in breakaway_events(load(f), dyn):
            e['file'] = os.path.basename(f); events.append(e)
    up = [e['net'] for e in events if e['direction'] > 0]
    dn = [-e['net'] for e in events if e['direction'] < 0]
    fs_pos = float(np.median(up)) if up else max(dyn['fc_pos'], 0.0)
    fs_neg = float(np.median(dn)) if dn else max(dyn['fc_neg'], 0.0)
    plant = dict(robot=4, gear_ratio='user_reported_10.4_reduction (robot 4; not the FGC_PIDF_Tuner mechanism)',
                 angle='sensor deg = V/Vmax*135', phase_deg=PHASE_DEG,
                 gravity_cos=dyn['gravity_cos'], gravity_constant=dyn['gravity_constant'],
                 fc_pos=max(dyn['fc_pos'], 0.0), fc_neg=max(dyn['fc_neg'], 0.0),
                 fs_pos=max(fs_pos, dyn['fc_pos'], 0.0), fs_neg=max(fs_neg, dyn['fc_neg'], 0.0),
                 kv=max(dyn['kv'], 1e-5), ka=max(dyn['ka'], 1e-6),
                 fit=dict((k, dyn[k]) for k in ('samples', 'up_samples', 'down_samples', 'rms', 'r2')),
                 raw_dynamic_fit=dyn, breakaway_events=events,
                 sources=[os.path.basename(f) for f in auto + brk])
    plant['tau_s'] = plant['ka'] / plant['kv']
    plant['kv_ff_theory'] = plant['kv']   # duty per deg/s: model-based velocity feedforward
    return plant


# ---------------------------------------------------------------- plant + Java law simulation
class Plant:
    """Rigid joint with gravity, direction-dependent static/kinetic friction, viscous kV and inertia kA."""
    def __init__(self, p, q0, scale=None):
        s = scale or {}
        self.gc = p['gravity_cos'] * s.get('g', 1); self.g0 = p['gravity_constant'] * s.get('g', 1)
        # 'stick' raises static above kinetic friction only: the stick-slip seen in run 11.
        self.fs = (p['fs_pos'] + s.get('fs', 0) + s.get('stick', 0), p['fs_neg'] + s.get('fs', 0) + s.get('stick', 0))
        self.fc = (p['fc_pos'] + s.get('fs', 0), p['fc_neg'] + s.get('fs', 0))
        self.kv = p['kv'] * s.get('kv', 1); self.ka = p['ka']
        self.q, self.v = q0, 0.0

    def gravity(self, q):
        return self.gc * math.cos(math.radians(q + PHASE_DEG)) + self.g0

    def step(self, u, dt, sub=10):
        h = dt / sub
        for _ in range(sub):
            net = u - self.gravity(self.q)
            if self.v == 0.0:
                if -self.fs[1] <= net <= self.fs[0]:
                    continue
                self.v = 1e-6 if net > 0 else -1e-6
            fric = self.fc[0] if self.v > 0 else -self.fc[1]
            acc = (net - fric - self.kv * self.v) / self.ka
            nv = self.v + acc * h
            if nv * self.v < 0:
                nv = 0.0      # friction stops the joint; re-test the static condition next substep
            self.q += 0.5 * (self.v + nv) * h; self.v = nv
        return self.q


def simulate(p, g, start, goal, seconds=4.0, scale=None, quantum=135.0 / 1024):
    """Port of ExtendTuneAutoCore.update() for one MOVE (no limits; output clipped to +/-1)."""
    plant = Plant(p, start, scale)
    target, integral, prev_err, vel, prev_ang = start, 0.0, 0.0, 0.0, start
    log = []
    for n in range(int(seconds / DT)):
        ang = round(plant.q / quantum) * quantum
        if n:
            vel += .35 * ((ang - prev_ang) / DT - vel)
        prev_ang = ang
        prev_target = target
        target += max(-g['slew'] * DT, min(g['slew'] * DT, goal - target))
        tv = (target - prev_target) / DT
        err = target - ang
        integral = max(-.5, min(.5, integral + err * DT))
        d = g['kd'] * (err - prev_err) / DT; prev_err = err
        fr = 0.0 if vel * math.copysign(1, err) > 15 else g['friction'] * max(-1, min(1, err / 2))
        ff = (p_gc(g, p) * math.cos(math.radians(ang + PHASE_DEG)) + g_g0(g, p) + fr
              + g['kv_up'] * max(0.0, tv) + g['kv_down'] * min(0.0, tv))
        limit = g.get('max_power', 1.0)   # only to re-create the recorded (limited) runs
        u = max(-limit, min(limit, ff + g['kp'] * err + g['ki'] * integral + d))
        plant.step(u, DT)
        log.append((n * DT, plant.q, plant.v, target, u))
    return np.array(log)


def p_gc(g, p): return g.get('gravity_cos', p['gravity_cos'])
def g_g0(g, p): return g.get('gravity_constant', p['gravity_constant'])


def score(log, start, goal, band=1.5):
    t, q, v, target, u = log.T
    sign = 1 if goal > start else -1
    err = goal - q
    inside = np.abs(err) <= band
    settle = next((t[k] for k in range(len(t)) if inside[k:].all()), None)
    over = max(0.0, float(np.max(sign * (q - goal))))
    final = float(abs(err[-1]))
    sat = float(np.mean(np.abs(u) >= .999))
    track = float(np.sqrt(np.mean((target - q) ** 2)))
    # Chatter: run 11 (kp 0.14) reversed the output between +1 and -0.4 during the move.
    s = np.sign(u[np.abs(u) > .05])
    reversals = int(np.sum(s[1:] != s[:-1]))
    variation = float(np.sum(np.abs(np.diff(u))))
    ok = settle is not None and over <= 3.0 and final <= band and reversals <= 2
    cost = ((settle if settle is not None else 99) + .5 * over + .5 * final + .02 * float(np.mean(u ** 2))
            + .1 * variation)
    return dict(ok=ok, settle_s=settle, overshoot_deg=over, final_err_deg=final, peak_dps=float(np.max(np.abs(v))),
                saturation=sat, tracking_rms_deg=track, output_reversals=reversals, output_variation=variation,
                cost=cost)


# FGC_PIDF_Tuner's 27 cases, widened on the fast side: the closed-loop replay of runs 07/09/10 is
# about 20-30 % slower than the robot (stick-slip surges), so kV x0.6 and friction -0.05 are added.
ROBUST = [dict(g=g, fs=f, kv=k) for g in (.93, 1, 1.07) for f in (-.05, -.03, 0, .03) for k in (.6, .8, 1, 1.25)]
MOVES = [(25.0, 71.0), (71.0, 25.0)]


def evaluate(p, g, robust=False):
    worst = None
    for start, goal in MOVES:
        for s in (ROBUST if robust else [None]):
            r = score(simulate(p, g, start, goal, scale=s), start, goal)
            r['move'] = f'{start:.0f}->{goal:.0f}'
            if worst is None or (not r['ok'], r['cost']) > (not worst['ok'], worst['cost']):
                worst = r
            if not r['ok'] and robust:
                return worst
    return worst


def search(p, base):
    grid = dict(kp=[.0175, .035, .05, .07, .1, .14, .2], ki=[0, .005, .02, .05], kd=[0, .0005, .001, .002],
                friction=[.5 * p['fs_pos'], p['fs_pos'], .5 * (p['fs_pos'] + p['fs_neg'])],
                kv_up=[0, .5 * p['kv'], p['kv'], 1.5 * p['kv']], kv_down=[0, .5 * p['kv'], p['kv']])
    rows = []
    keys = list(grid)
    for vals in itertools.product(*(grid[k] for k in keys)):
        g = dict(base, **dict(zip(keys, vals)))
        r = evaluate(p, g)
        if r['ok']:
            rows.append((r['cost'], g, r))
    rows.sort(key=lambda x: x[0])
    robust = []
    for cost, g, r in rows[:40]:
        rr = evaluate(p, g, robust=True)
        robust.append(dict(gains=g, nominal=r, worst_case=rr))
    robust.sort(key=lambda x: (not x['worst_case']['ok'], x['worst_case']['cost']))
    for c in robust[:10]:
        c['loop_period'] = loop_check(p, c['gains'])
    # Prefer candidates that also pass at 30/40 ms loops (FGC_PIDF_Tuner found kd fails there).
    robust[:10] = sorted(robust[:10], key=lambda x: (not x['worst_case']['ok'],
                                                     not all(v['ok'] for v in x['loop_period'].values()),
                                                     x['worst_case']['cost']))
    return dict(grid=grid, nominal_pass=len(rows), candidates=robust[:10])


def loop_check(p, g):
    """Nominal plant at slower loops; the Java law measures dt, the plant sees the same period."""
    global DT
    saved, res = DT, {}
    try:
        for ms in (30, 40):
            DT = ms / 1000
            r = evaluate(p, g)
            res[f'{ms}ms'] = dict(ok=r['ok'], settle_s=r['settle_s'], overshoot_deg=r['overshoot_deg'])
    finally:
        DT = saved
    return res


# ---------------------------------------------------------------- closed-loop check against the recorded Auto runs
# Settings of each recorded run (handoff README / manifest). Gains are the Hub file of that session.
RECORDED = {
    '07_': dict(max_power=.35, kv_up=0, fault=''),
    '08_': dict(max_power=.50, kv_up=0, fault='OVERSPEED'),
    '09_': dict(max_power=.50, kv_up=0, fault=''),
    '10_': dict(max_power=.50, kv_up=.010, fault='OVERSPEED'),
    # First no-limit run with the r4_pidf candidate: reached 70.4 deg in 1.8 s with stick-slip chatter.
    '11_': dict(max_power=1.0, kp=.14, ki=.005, kd=.001, friction=.126, kv_up=.0028,
                gravity_cos=.202, gravity_constant=.045, fault=''),
}
SESSION = dict(slew=27.0, kp=.0175, ki=.005, kd=0, friction=.123, kv_down=0,
               gravity_cos=.0638, gravity_constant=.0694)


def replay(p, folder):
    """Re-run the Java law with each recorded run's settings on the identified plant and compare the
    first MOVE: time to arrive within 1.5 deg, and peak speed (recorded runs stopped above 45 deg/s)."""
    out = []
    for f in sorted(glob.glob(os.path.join(folder, '*.csv'))):
        key = os.path.basename(f)[:3]
        if key not in RECORDED or not key.startswith(REPLAY_FILES):
            continue
        d = load(f); a = d[(d.state == 'ACTIVE') & (d.segment == 'MOVE')]
        if a.empty:
            continue
        # The 25<->71 deg MOVE: the station farthest from where its MOVE started.
        firsts = a.groupby('station_deg').angle_deg.first()
        station = (firsts - firsts.index.to_series()).abs().idxmax()
        a = a[a.station_deg == station]
        start, goal = float(a.angle_deg.iloc[0]), float(a.station_deg.iloc[0])
        t = a.t.to_numpy() - a.t.iloc[0]
        # 90 % of the commanded travel: the recorded runs stopped short of the 1.5 deg arrival band.
        sign = 1 if goal > start else -1
        mark = start + .9 * (goal - start)
        rec_in = np.nonzero(sign * (a.angle_deg.to_numpy() - mark) >= 0)[0]
        g = dict(SESSION, **{k: v for k, v in RECORDED[key].items() if k != 'fault'})
        log = simulate(p, g, start, goal, seconds=max(4.0, float(t[-1]) + 1))
        sim_in = np.nonzero(sign * (log[:, 1] - mark) >= 0)[0]
        out.append(dict(file=os.path.basename(f), move=f'{start:.1f}->{goal:.0f}',
                        recorded_fault=RECORDED[key]['fault'], recorded_end_deg=float(a.angle_deg.iloc[-1]),
                        recorded_reach90_s=float(t[rec_in[0]]) if len(rec_in) else None,
                        simulated_reach90_s=float(log[sim_in[0], 0]) if len(sim_in) else None,
                        recorded_peak_dps=float(a.velocity_dps.abs().max()),
                        simulated_peak_dps=float(np.max(np.abs(log[:, 2])))))
    return out


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('identify'); s.add_argument('folder'); s.add_argument('--out', default='r4_plant.json')
    s = sub.add_parser('replay'); s.add_argument('plant'); s.add_argument('folder')
    s = sub.add_parser('search'); s.add_argument('plant'); s.add_argument('--out', default='r4_gains.json')
    s.add_argument('--slew', type=float, default=27.0)
    args = ap.parse_args()
    if args.cmd == 'identify':
        p = identify(args.folder)
        json.dump(p, open(args.out, 'w'), indent=2)
        print(json.dumps({k: p[k] for k in ('gravity_cos', 'gravity_constant', 'fs_pos', 'fs_neg', 'fc_pos',
                                             'fc_neg', 'kv', 'ka', 'tau_s', 'fit')}, indent=2))
    elif args.cmd == 'replay':
        print(json.dumps(replay(json.load(open(args.plant)), args.folder), indent=2))
    else:
        p = json.load(open(args.plant))
        res = search(p, dict(slew=args.slew))
        current = dict(slew=args.slew, kp=.0175, ki=.005, kd=0, friction=.123, kv_up=.003, kv_down=0,
                       gravity_cos=.0638, gravity_constant=.0694)
        res['current_hub_settings'] = dict(gains=current, nominal=evaluate(p, current),
                                           worst_case=evaluate(p, current, robust=True))
        json.dump(res, open(args.out, 'w'), indent=2)
        print('nominal pass:', res['nominal_pass'])
        for c in res['candidates'][:5]:
            g = c['gains']; w = c['worst_case']
            print({k: round(g[k], 5) for k in ('kp', 'ki', 'kd', 'friction', 'kv_up', 'kv_down')},
                  'worst', w['move'], 'ok' if w['ok'] else 'NG', w['settle_s'], round(w['overshoot_deg'], 2),
                  round(w['peak_dps'], 1), 'loops', {k: v['ok'] for k, v in c['loop_period'].items()})
        c = res['current_hub_settings']
        print('current Hub settings:', 'nominal', c['nominal']['ok'], c['nominal']['settle_s'],
              'worst', c['worst_case']['ok'], c['worst_case']['move'])


if __name__ == '__main__':
    sys.exit(main())
