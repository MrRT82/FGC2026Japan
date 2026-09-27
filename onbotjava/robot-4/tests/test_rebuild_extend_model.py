import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from rebuild_extend_model import (advance, fit_sliding, fit_static, gravity_voltage,
                                  breakaway_bracket, rebuild, prepare_interval, replay)
from tune_extend_pid import tune, simulate_candidate


MODEL = dict(kA_V_per_deg_s2=.02, kV_V_per_deg_s=.03, kG_V=.7, phase_deg=17,
             Fc_positive_V=.35, Fc_negative_V=.45, Fs_positive_V=.8, Fs_negative_V=.9,
             Fs_positive_interval_V=[.78,.82], Fs_negative_interval_V=[.88,.92],
             angle_range_deg=[-30,100], effort_range_V=[-12,12], voltage_range_V=[11.5,12.5], speed_range_dps=[-40,40])
SETTINGS = dict(moving_dps=2, position_noise_deg=.15, motion_displacement_deg=.3,
                max_ramp_V_per_s=3, effort_uncertainty_V=.05,
                max_replay_rmse_deg=1, max_replay_error_deg=3, max_onset_error_s=.2,
                max_sliding_rmse_V=.3)
GEOMETRY = dict(mechanism='single_rigid_joint', sensor_scale=1, sensor_offset_deg=0,
                gravity_phase_deg=17, command_sign=1, source='synthetic fixture')


def sliding(phase=0):
    t=np.arange(0,24,.02)
    q=35+25*np.sin(.6*t+phase)+8*np.sin(1.7*t+phase)
    v=15*np.cos(.6*t+phase)+13.6*np.cos(1.7*t+phase)
    a=-9*np.sin(.6*t+phase)-23.12*np.sin(1.7*t+phase)
    u=.02*a+.03*v+gravity_voltage(MODEL,q)+np.where(v>0,.35,-.45)
    return dict(t=t,q=q,u=u,entry={})


def onset(direction=1, shift=0):
    t=np.arange(0,2,.02)
    q=35+direction*np.maximum(0,t-.9)*4
    effort=(.8 if direction==1 else .9)+.4*(t-.9)+shift
    u=gravity_voltage(MODEL,q)+direction*effort
    return dict(t=t,q=q,u=u,entry=dict(start_s=0,end_s=2,last_still_s=.8,first_motion_s=1.,direction=direction))


def generated_replay(direction=1):
    t=np.arange(0,6,.02); q=35.; v=0.; angles=[]; volts=[]
    for stamp in t:
        # A finite pulse then gravity-only effort lets Coulomb friction stop the arm.
        effort=direction*1.0 if .8<=stamp<1.5 else 0.
        u=gravity_voltage(MODEL,q)+effort
        angles.append(q); volts.append(u)
        for _ in range(20): q,v=advance(q,v,u,.001,MODEL)
    return dict(t=t,q=np.array(angles),u=np.array(volts),entry={})


class NonlinearTests(unittest.TestCase):
    def test_csv_to_model_to_pid_pipeline(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            files=[]
            for role in ('train','validation'):
                signals=[('sliding',sliding(0 if role=='train' else .4))]
                signals += [('breakaway',onset(s)) for s in (1,-1)] * (2 if role=='train' else 1)
                if role=='validation': signals += [('replay',generated_replay(s)) for s in (1,-1)]
                rows,intervals,offset=[],[],0.
                for kind,run in signals:
                    entry=dict(run['entry'],kind=kind,start_s=offset,end_s=offset+float(run['t'][-1]))
                    if kind=='breakaway':
                        entry['last_still_s']+=offset; entry['first_motion_s']+=offset
                    intervals.append(entry)
                    for t,q,u in zip(run['t'],run['q'],run['u']):
                        stamp=float(t+offset)*1000
                        rows.append(dict(t_ms=stamp,angle_t_ms=stamp,output_t_ms=stamp,angle_deg=q,
                                         sent_out=u/12,state='ACTIVE',fault='',hub_0_voltage_V_seq=len(rows)+1,
                                         hub_0_voltage_V_age_ms=0,hub_0_voltage_V_read_ms=1,
                                         hub_0_voltage_V_ok=1,hub_0_voltage_V=12))
                    offset+=float(run['t'][-1])+1
                path=root/f'{role}.csv'
                with path.open('w',newline='') as f:
                    w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
                path.with_suffix('.properties').write_text('robot=4\nballs=0\nmotor_model=test\ngear_ratio=test\nmin_angle_deg=1\nmax_angle_deg=75\nmax_power=0.25\n')
                files.append(dict(csv=path.name,role=role,intervals=intervals))
            spec=dict(geometry=GEOMETRY,identification=SETTINGS,balls=0,motor_model='test',gear_ratio='test',
                      motor_hub_voltage_signal='hub_0_voltage_V',files=files)
            manifest=root/'manifest.json'; manifest.write_text(json.dumps(spec))
            result=rebuild(manifest,root/'model')
            self.assertEqual(result['status'],'model_validation_passed',result)
            search=dict(reference_voltage_V=12,kp=[.01],ki=[.001],kd=[.001],friction=[.04,.06,.12],
                        max_rmse_deg=10,max_final_error_deg=5,max_peak_error_deg=10,
                        cases=[dict(start_angle_sensor_deg=35,step_deg=s) for s in (5,-5)])
            search_path=root/'search.json'; search_path.write_text(json.dumps(search))
            pid=tune(root/'model'/'model_report.json',root/'train.properties',search_path,root/'pid.json')
            self.assertEqual(pid['status'],'offline_candidates',pid)
            self.assertTrue(pid['candidates'])
            # File-content leakage across roles is rejected, even under another name.
            spec['files'][1]['csv']='train.csv'; manifest.write_text(json.dumps(spec))
            bad=rebuild(manifest,root/'bad')
            self.assertIn('hash reused',bad['reason'])

    def test_causal_voltage_and_missing_sample(self):
        rows=[]
        for i in range(100):
            rows.append(dict(t_ms=str(i*20),angle_t_ms=str(i*20),output_t_ms=str(i*20+5),
                             angle_deg='35',sent_out='.1',state='ACTIVE',fault='',hub_0_voltage_V_seq=str(i+1),
                             hub_0_voltage_V_age_ms='0',hub_0_voltage_V_read_ms='1',hub_0_voltage_V_ok='1',hub_0_voltage_V='12'))
        run=prepare_interval(rows,dict(start_s=0,end_s=2),GEOMETRY,'hub_0_voltage_V')
        self.assertAlmostEqual(run['t'][0],.005)
        np.testing.assert_allclose(run['u'],1.2)
        rows[30]['hub_0_voltage_V_ok']='0'
        with self.assertRaisesRegex(ValueError,'Invalid/stale'):
            prepare_interval(rows,dict(start_s=0,end_s=2),GEOMETRY,'hub_0_voltage_V')

    def test_static_hold_and_directional_breakaway(self):
        q=35.; g=gravity_voltage(MODEL,q)
        self.assertEqual(advance(q,0,g+.79,.01,MODEL),(q,0))
        self.assertEqual(advance(q,0,g-.89,.01,MODEL),(q,0))
        self.assertGreater(advance(q,0,g+.81,.01,MODEL)[1],0)
        self.assertLess(advance(q,0,g-.91,.01,MODEL)[1],0)

    def test_stopping_does_not_chatter(self):
        q,v=35.,.01
        q2,v2=advance(q,v,gravity_voltage(MODEL,q),.02,MODEL)
        self.assertEqual(v2,0)
        self.assertGreater(q2,q)

    def test_separate_dynamic_terms_recovered(self):
        result=fit_sliding([sliding()],17,2)
        for k in ('kA_V_per_deg_s2','kV_V_per_deg_s','kG_V','Fc_positive_V','Fc_negative_V'):
            self.assertAlmostEqual(result[k],MODEL[k],delta=.01)

    def test_static_requires_both_directions_and_repetition(self):
        with self.assertRaisesRegex(ValueError,'2 independent'):
            fit_static([onset(1)],dict(MODEL),SETTINGS)
        model=dict(MODEL)
        fit_static([onset(1),onset(1,.01),onset(-1),onset(-1,.01)],model,SETTINGS)
        self.assertLess(model['Fs_positive_interval_V'][0],.8)
        self.assertGreater(model['Fs_positive_interval_V'][1],.8)

    def test_contradictory_static_brackets_rejected(self):
        with self.assertRaisesRegex(ValueError,'inconsistent'):
            fit_static([onset(1),onset(1,1),onset(-1),onset(-1)],dict(MODEL),SETTINGS)

    def test_moving_prehold_rejected(self):
        run=onset(); run['q'][:45]+=np.linspace(0,10,45)
        with self.assertRaisesRegex(ValueError,'not stationary'):
            breakaway_bracket(run,MODEL,SETTINGS)

    def test_independent_replay_and_wrong_static_threshold(self):
        run=generated_replay()
        result,_=replay(run,MODEL,SETTINGS)
        self.assertTrue(result['pass_angle'])
        bad=dict(MODEL,Fs_positive_V=2)
        result,_=replay(run,bad,SETTINGS)
        self.assertFalse(result['pass_angle'])
        self.assertIsNone(result['onset_error_s'])

    def test_missing_cad_produces_actionable_report(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'manifest.json'
            p.write_text(json.dumps(dict(geometry=dict(GEOMETRY,sensor_scale=None))))
            r=rebuild(p,Path(d)/'out')
            self.assertEqual(r['status'],'needs_data')
            self.assertIn('sensor_scale',r['reason'])
            self.assertFalse(r['gain_generation_enabled'])

    def test_stage_two_blocks_unvalidated_model(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); p=root/'model.json'
            p.write_text(json.dumps(dict(schema='robot4-stick-slip-v2',status='rebuild_required')))
            with self.assertRaisesRegex(ValueError,'not passed'):
                tune(p,root/'unused',root/'unused',root/'out')

    def test_pid_and_friction_reference_simulation(self):
        cfg=dict(min_angle_deg=1,max_angle_deg=75,max_power=.25)
        candidate=dict(kp=.0175,ki=.001,kd=.003,friction=.12,gravity_cos=.7/12,
                       gravity_phase_deg=17,gravity_constant=0)
        scores,reason=simulate_candidate(MODEL,GEOMETRY,cfg,candidate,dict(start_angle_sensor_deg=35,step_deg=5),12)
        self.assertIsNone(reason)
        self.assertIsNotNone(scores)


if __name__=='__main__': unittest.main()
