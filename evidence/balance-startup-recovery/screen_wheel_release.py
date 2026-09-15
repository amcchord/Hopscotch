import sys,json,statistics,itertools,importlib.util
from pathlib import Path
from dataclasses import replace
out=Path('evidence/balance-startup-recovery')
text=(out/'wheel-screen-simulator.py').read_text()
text=text.replace('    recovery_ki: float = 1.0','    recovery_boost_ms: float = 800.0\n    recovery_ki: float = 1.0')
text=text.replace('change=clampf(c.recovery_ki*vel_err,-c.recovery_rate,c.recovery_rate)*dt','ki=c.recovery_ki if not self.ramp_complete and now_ms-self.startup_ms<c.recovery_boost_ms else c.vel_sp_ki\n            change=clampf(ki*vel_err,-c.recovery_rate,c.recovery_rate)*dt')
text=text.replace('elif abs_err <= c.vel_sp_knee or not self.ramp_complete or wheel_active:', 'elif abs_err <= c.vel_sp_knee or not self.ramp_complete:')
(out/'wheel-release-simulator.py').write_text(text)
spec=importlib.util.spec_from_file_location('wheel_release',out/'wheel-release-simulator.py');s=importlib.util.module_from_spec(spec);sys.modules[spec.name]=s;spec.loader.exec_module(s)
base=s.FirmwareConfig(**json.loads((out/'candidate-config.json').read_text()));base=replace(base,startup_recovery=False)
plant=s.load_fitted_params(Path('evidence/balance-review/model-fit-speed.json'))
results=[]
variants=[('baseline',base)]
variants += [(f'v{v}_ki{ki}_ms{ms}',replace(base,wheel_recovery=True,startup_speed=v,recovery_ki=ki,recovery_boost_ms=ms)) for v,ki,ms in itertools.product((1.,1.5),(1.,2.),(400.,800.,5000.))]
for name,cfg in variants:
 cases=[]
 for delta,af,tau,offset in itertools.product((-1.9,-3.5,-4.5),(.7,1.4),(.01,.06),(-.4,.4)):
  p=replace(plant,A=plant.A*af,B=plant.B*.66,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1.,fb_hold_ticks=1,vel_meas_noise=.1)
  r=s.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=1)
  early=r.fell and (r.ramp_complete_s is None or r.t[-1]<r.ramp_complete_s+3)
  cases.append(dict(delta=delta,af=af,tau=tau,offset=offset,early=early,late=r.fell and not early,ramp=r.ramp_complete_s,peak=abs(r.peak_drift_standup)))
 summary=dict(name=name,early=sum(c['early'] for c in cases),late=sum(c['late'] for c in cases),median_peak=statistics.median(c['peak'] for c in cases))
 print(summary,flush=True);results.append(dict(summary=summary,cases=cases))
(out/'wheel-release-screening.json').write_text(json.dumps(results,indent=2)+'\n')
