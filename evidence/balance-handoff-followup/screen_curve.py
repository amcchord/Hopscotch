import sys,json,statistics
from pathlib import Path
from dataclasses import replace
sys.path.insert(0,str(Path.cwd()/'scripts'))
import balance_sim as s
base=s.FirmwareConfig(**json.loads(Path(__file__).with_name('baseline-config.json').read_text()))
plant=s.load_fitted_params(Path('evidence/balance-review/model-fit-speed.json'))
results=[]
for curve in (1.9,3.,4.,4.5):
 cfg=replace(base,sp_curve=((0.,84.),(.5,84.-curve/2),(1.,84.-curve)))
 cases=[]
 for delta in (-1.9,-3.5,-4.5):
  for af in (.7,1.,1.4):
   for tau in (.005,.02,.06):
    for offset in (-.4,.4):
     p=replace(plant,A=plant.A*af,B=plant.B*.66,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1.,fb_hold_ticks=1,vel_meas_noise=.1)
     r=s.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=1)
     early=r.fell and (r.ramp_complete_s is None or r.t[-1]<r.ramp_complete_s+3)
     cases.append(dict(delta=delta,af=af,tau=tau,offset=offset,early=early,late=r.fell and not early,ramp=r.ramp_complete_s,peak=abs(r.peak_drift_standup)))
 summary=dict(curve=curve,cases=len(cases),early=sum(c['early'] for c in cases),late=sum(c['late'] for c in cases),median_peak=statistics.median(c['peak'] for c in cases if not c['early']) if any(not c['early'] for c in cases) else None)
 print(summary,flush=True);results.append(dict(summary=summary,cases=cases))
Path('evidence/balance-handoff-followup/curve-screening.json').write_text(json.dumps(results,indent=2)+'\n')
