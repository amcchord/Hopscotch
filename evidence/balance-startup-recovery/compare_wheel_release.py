import sys,json,statistics,itertools,importlib.util
from pathlib import Path
from dataclasses import replace
out=Path('evidence/balance-startup-recovery')
text=(out/'wheel-release-simulator.py').read_text().replace('    wheel_recovery: bool = False','    recovery_hold_position: bool = False\n    wheel_recovery: bool = False')
text=text.replace('''                self.recovery_done=True
                wheel_active=False''','''                self.recovery_done=True
                wheel_active=False
                if c.recovery_hold_position:
                    self.wheel_start_pos=wheel_pos
                    meas_drift=0.''')
# Capture the measured wheel position as the settled holding point.
(out/'wheel-final-screen-simulator.py').write_text(text)
spec=importlib.util.spec_from_file_location('wheel_final_screen',out/'wheel-final-screen-simulator.py');s=importlib.util.module_from_spec(spec);sys.modules[spec.name]=s;spec.loader.exec_module(s)
base=s.FirmwareConfig(**json.loads((out/'candidate-config.json').read_text()));base=replace(base,startup_recovery=False)
plant=s.load_fitted_params(Path('evidence/balance-review/model-fit-speed.json'))
results=[]
variants=[('baseline',base)]
variants += [(f'wheel_ms{ms}_hold{hold}',replace(base,wheel_recovery=True,startup_speed=1.,recovery_ki=1.,recovery_boost_ms=ms,recovery_hold_position=hold)) for ms,hold in ((800.,False),(800.,True))]
for name,cfg in variants:
 cases=[]
 for delta,af,bf,tau,offset,seed in itertools.product((-1.9,-3.5,-4.5),(.7,1.,1.4),(.7,1.,1.4),(.01,.03,.06),(-.8,.8),(1,2)):
  p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1.,fb_hold_ticks=1,vel_meas_noise=.1)
  r=s.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed)
  early=r.fell and (r.ramp_complete_s is None or r.t[-1]<r.ramp_complete_s+3)
  cases.append(dict(delta=delta,af=af,bf=bf,tau=tau,offset=offset,seed=seed,early=early,late=r.fell and not early,ramp=r.ramp_complete_s,peak=abs(r.peak_drift_standup)))
 summary=dict(name=name,cases=len(cases),early=sum(c['early'] for c in cases),late=sum(c['late'] for c in cases),no_ramp=sum(c['ramp'] is None for c in cases),median_peak=statistics.median(c['peak'] for c in cases))
 print(summary,flush=True);results.append(dict(summary=summary,cases=cases))
(out/'wheel-final-comparison.json').write_text(json.dumps(results,indent=2)+'\n')
