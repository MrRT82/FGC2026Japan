"""Stage 2: PID + friction search gated on stage-1 stick/slip validation.

Simulates the existing ExtendTuneCore control law, including its scalar friction
compensation, integral clamp, error derivative, slew and motion guards.
Exports offline candidates only, never writes Hub properties or runs hardware.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

from extend_analysis import properties, save_json
from rebuild_extend_model import advance


def clip(value, low, high): return max(low, min(high, value))


def simulate_candidate(model, geometry, cfg, candidate, case, volts):
    dt = .02
    sensor = case['start_angle_sensor_deg']
    physical = sensor * geometry['sensor_scale'] + geometry['sensor_offset_deg']
    v, filtered_v, target, integral, previous_error = 0., 0., sensor, 0., 0.
    center, stall_angle, stall_seconds = sensor, sensor, 0.
    error_sum, saturated, final_error, max_speed, peak_error = 0., 0, 0., 0., 0.
    low, high = float(cfg['min_angle_deg']), float(cfg['max_angle_deg'])
    power_limit = float(cfg['max_power'])
    slew_dps = float(cfg.get('slew_dps', '10'))
    for n in range(400):
        goal = center + (case['step_deg'] if n >= 50 else 0)
        goal = clip(goal, max(low + 1, center - 10), min(high - 1, center + 10))
        target += clip(goal - target, -slew_dps*dt, slew_dps*dt)
        error = target - sensor
        integral = clip(integral + error * dt, -.5, .5)
        friction = 0. if filtered_v * (1 if error > 0 else -1 if error < 0 else 0) > 15 else candidate['friction'] * clip(error / 2, -1, 1)
        ff = candidate['gravity_cos'] * math.cos(math.radians(sensor + candidate['gravity_phase_deg'])) + candidate['gravity_constant'] + friction
        requested = ff + candidate['kp'] * error + candidate['ki'] * integral + candidate['kd'] * (error - previous_error) / dt
        previous_error = error
        sent = clip(requested, -power_limit, power_limit)
        if abs(sensor-stall_angle) >= 1 or abs(error) < 3 or abs(sent) < .1:
            stall_angle, stall_seconds = sensor, 0.
        else:
            stall_seconds += dt
            if stall_seconds > .5: return None, 'NO_MOTION'
        u = geometry['command_sign'] * sent * volts
        if not model['effort_range_V'][0] <= u <= model['effort_range_V'][1]:
            return None, 'EFFORT_OUTSIDE_IDENTIFIED_RANGE'
        old = sensor
        for _ in range(10):
            physical, v = advance(physical, v, u, dt/10, model)
        sensor = (physical - geometry['sensor_offset_deg']) / geometry['sensor_scale']
        filtered_v += .35 * ((sensor-old)/dt-filtered_v)
        if not math.isfinite(sensor) or sensor <= low or sensor >= high:
            return None, 'ANGLE_LIMIT'
        if abs(filtered_v) > 45: return None, 'OVERSPEED'
        if abs(sensor-center) > 10: return None, 'TRAVEL_LIMIT'
        if not model['speed_range_dps'][0] <= v <= model['speed_range_dps'][1]:
            return None, 'SPEED_OUTSIDE_IDENTIFIED_RANGE'
        if not model['angle_range_deg'][0] <= physical <= model['angle_range_deg'][1]:
            return None, 'ANGLE_OUTSIDE_IDENTIFIED_RANGE'
        error_sum += (target-sensor)**2
        peak_error = max(peak_error, abs(target-sensor))
        saturated += int(abs(requested) > power_limit)
        final_error = abs(goal-sensor)
        max_speed = max(max_speed, abs(filtered_v))
    rmse = math.sqrt(error_sum/400)
    return dict(rmse_deg=rmse, final_error_deg=final_error, peak_error_deg=peak_error,
                saturation_fraction=saturated/400, max_speed_dps=max_speed,
                score=rmse+final_error+5*saturated/400), None


def tune(model_path, settings_path, search_path, out):
    save_json(out, dict(schema='robot4-pid-candidates-v2', status='not_generated', candidates=[]))
    report = json.loads(model_path.read_text(encoding='utf-8-sig'))
    if (report.get('schema') != 'robot4-stick-slip-v2' or report.get('status') != 'model_validation_passed'
            or not report.get('validation') or not all(x.get('passed') for x in report['validation'])):
        raise ValueError('Stage 1 has not passed independent validation; no PID search permitted')
    model, geometry = report['model'], report['geometry']
    if abs(abs(geometry['sensor_scale'])-1) > 1e-6:
        raise ValueError('Existing Java gravity cosine cannot represent this CAD scale; update controller mapping first')
    if geometry['sensor_scale'] * geometry['command_sign'] <= 0:
        raise ValueError('Existing Java positive PID sign is incompatible with this geometry/command mapping')
    cfg = properties(settings_path)
    if cfg.get('robot') != '4' or any(cfg.get(k) != str(v) for k, v in report['load'].items()):
        raise ValueError('Stage 2 configuration robot/load/gearing differs from identified model')
    values = [float(cfg[k]) for k in ('min_angle_deg', 'max_angle_deg', 'max_power')]
    if not all(map(math.isfinite, values)) or not (0 <= values[0] < values[1] <= 88 and 0 < values[2] <= .35):
        raise ValueError('Invalid Java motion limits')
    if not math.isfinite(float(cfg.get('slew_dps', '10'))) or not 0 < float(cfg.get('slew_dps', '10')) <= 10:
        raise ValueError('Invalid Java slew_dps')
    search = json.loads(search_path.read_text(encoding='utf-8-sig'))
    for k in ('max_rmse_deg', 'max_final_error_deg', 'max_peak_error_deg'):
        if not math.isfinite(float(search[k])) or search[k] <= 0: raise ValueError('Positive acceptance limits required')
    if not search['cases'] or {int(math.copysign(1, c['step_deg'])) for c in search['cases']} != {-1, 1}:
        raise ValueError('Positive and negative step cases required')
    for c in search['cases']:
        if not all(math.isfinite(float(c[k])) for k in ('start_angle_sensor_deg', 'step_deg')):
            raise ValueError('Invalid scenario')
        if not 0 < abs(c['step_deg']) <= 10: raise ValueError('Use nonzero steps up to 10 degrees')
        if not values[0]+1 <= c['start_angle_sensor_deg']+c['step_deg'] <= values[1]-1:
            raise ValueError('Scenario goal outside Java limits')
        if not values[0]+1 <= c['start_angle_sensor_deg'] <= values[1]-1:
            raise ValueError('Scenario start outside Java limits')
    grids = [search[k] for k in ('kp', 'ki', 'kd', 'friction')]
    if any(not g or any(not math.isfinite(float(x)) or x < 0 for x in g) for g in grids):
        raise ValueError('Search grids must be finite nonnegative numbers')
    if math.prod(map(len, grids)) > 1000: raise ValueError('Maximum 1000 grid candidates')
    reference = float(search['reference_voltage_V'])
    if not model['voltage_range_V'][0] <= reference <= model['voltage_range_V'][1]:
        raise ValueError('Reference voltage outside identification coverage')
    ranked, rejections = [], {}
    for gains in itertools.product(*grids):
        candidate = dict(zip(('kp', 'ki', 'kd', 'friction'), gains))
        candidate.update(gravity_cos=geometry['command_sign']*model['kG_V']/reference,
                         gravity_phase_deg=geometry['sensor_scale']*(geometry['sensor_offset_deg']+model['phase_deg']),
                         gravity_constant=0.)
        results, failure = [], None
        for pos, neg, volts, case in itertools.product(model['Fs_positive_interval_V'], model['Fs_negative_interval_V'],
                                                     sorted(set(model['voltage_range_V'])), search['cases']):
            alternative = dict(model, Fs_positive_V=pos, Fs_negative_V=neg)
            metrics, failure = simulate_candidate(alternative, geometry, cfg, candidate, case, volts)
            if failure: break
            if (metrics['rmse_deg'] > search['max_rmse_deg'] or metrics['final_error_deg'] > search['max_final_error_deg']
                    or metrics['peak_error_deg'] > search['max_peak_error_deg']):
                failure = 'TRACKING_ACCEPTANCE'; break
            results.append(dict(case=case, voltage_V=volts, Fs_positive_V=pos, Fs_negative_V=neg, **metrics))
        if failure:
            rejections[failure] = rejections.get(failure, 0) + 1
        else:
            ranked.append(dict(parameters=candidate, worst_score=max(x['score'] for x in results), scenarios=results))
    result = dict(schema='robot4-pid-candidates-v2', model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
                  settings_sha256=hashlib.sha256(settings_path.read_bytes()).hexdigest(), search=search,
                  controller='Existing ExtendTuneCore: PID + scalar error-shaped friction; integral clamp +/-0.5',
                  status='offline_candidates' if ranked else 'no_supported_candidate',
                  candidates=sorted(ranked, key=lambda r:r['worst_score'])[:10], rejected=rejections,
                  limitations=['No electrical/current trip or hard-real-time SDK latency prediction.',
                               'Threshold intervals and observed voltage endpoints tested; other parameter uncertainty not bounded.',
                               'Java has only one friction gain; identified asymmetric static/dynamic friction is in the plant. If no adequate candidate exists, revise Java compensation rather than force PID.',
                               'No automatic deployment or claim of optimality. Validate new gains on supervised hardware trials.'])
    save_json(out, result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('model',type=Path); p.add_argument('settings',type=Path); p.add_argument('search',type=Path)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args(); print(tune(a.model,a.settings,a.search,a.out)['status'])


if __name__ == '__main__': main()
