import sys,json,statistics
from pathlib import Path
from dataclasses import replace
sys.path.insert(0,str(Path.cwd()/'scripts'))
import balance_sim as s
out=Path('evidence/balance-standup-fix')
base=s.FirmwareConfig(**json.loads((out/'baseline-config.json').read_text()))
new=replace(s.current_firmware_config(),arm_return_acceleration=12.)
plant=s.load_fitted_params(Path('evidence/balance-review/model-fit-speed.json'))
results=[]
for name,cfg in [('flashed_before',base),('candidate',new)]:
    cases=[]
    for af in (.7,1.,1.4):
      for bf in (.66,1.,1.33):
       for delta in (-1.9,-3.3,-4.5):
        for offset in (-.8,.8):
         for seed in (1,2):
          p=replace(plant,A=plant.A*af,B=plant.B*bf,eq_tip_delta=delta,feedback_velocity_scale=(.66 if name=='flashed_before' else 1.),vel_meas_noise=plant.vel_meas_noise*(1 if name=='flashed_before' else 50/33))
          r=s.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed)
          early=r.fell and (r.ramp_complete_s is None or r.t[-1]<r.ramp_complete_s+3)
          cases.append(dict(af=af,bf=bf,delta=delta,offset=offset,seed=seed,early=early,late=r.fell and not early,ramp=r.ramp_complete_s,peak=abs(r.peak_drift_standup)))
    summary=dict(name=name,cases=len(cases),early=sum(c['early'] for c in cases),late=sum(c['late'] for c in cases),median_peak=statistics.median(c['peak'] for c in cases if not c['early']))
    print(summary,flush=True);results.append(dict(summary=summary,cases=cases))
(out/'final-simulation.json').write_text(json.dumps(results,indent=2)+'\n')
