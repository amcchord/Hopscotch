import sys,json,statistics,itertools
from pathlib import Path
from dataclasses import replace,asdict
sys.path.insert(0,str(Path.cwd()/'scripts'))
import balance_sim as s
out=Path('evidence/balance-startup-recovery')
base=replace(s.current_firmware_config(),startup_recovery=False)
plant=s.load_fitted_params(Path('evidence/balance-review/model-fit-speed.json'))
results=[]
for name,cfg in [('installed',base),('early_wheel_recovery',s.current_firmware_config())]:
 cases=[]
 for delta,af,bf,tau,offset,seed in itertools.product((-1.9,-3.5,-4.5),(.7,1.,1.4),(.7,1.,1.4),(.01,.03,.06),(-.8,.8),(1,2)):
  p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1.,fb_hold_ticks=1,vel_meas_noise=.1)
  r=s.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed)
  early=r.fell and (r.ramp_complete_s is None or r.t[-1]<r.ramp_complete_s+3)
  cases.append(dict(delta=delta,af=af,bf=bf,tau=tau,offset=offset,seed=seed,early=early,late=r.fell and not early,ramp=r.ramp_complete_s,peak=abs(r.peak_drift_standup)))
 summary=dict(name=name,cases=len(cases),early=sum(c['early'] for c in cases),late=sum(c['late'] for c in cases),no_ramp=sum(c['ramp'] is None for c in cases),median_peak=statistics.median(c['peak'] for c in cases))
 print(summary,flush=True);results.append(dict(summary=summary,config=asdict(cfg),cases=cases))
old,new=results
new_failures=sum(not(a['early'] or a['late']) and (b['early'] or b['late']) for a,b in zip(old['cases'],new['cases']))
rescues=sum((a['early'] or a['late']) and not(b['early'] or b['late']) for a,b in zip(old['cases'],new['cases']))
print(dict(new_failures=new_failures,rescued_cases=rescues),flush=True)
(out/'final-model-comparison.json').write_text(json.dumps(dict(results=results,new_failures=new_failures,rescued_cases=rescues),indent=2)+'\n')
