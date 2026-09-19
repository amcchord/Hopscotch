"""Small screening experiment only; production source and firmware unchanged."""
import importlib.util,sys,itertools,json,statistics,subprocess
from pathlib import Path
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
# Freeze the two-success baseline: later production edits must not silently
# change the meaning of this historical, ungated multiplier experiment.
FROZEN='75318bc7eac6b9df5a4bac68f4378ef1102b2838'
frozen_root=ROOT/'output/settling-review-baseline'
(frozen_root/'src').mkdir(parents=True,exist_ok=True)
(frozen_root/'scripts').mkdir(exist_ok=True)
source=subprocess.check_output(['git','show',f'{FROZEN}:scripts/balance_sim.py'],cwd=ROOT,text=True)
(frozen_root/'src/config.h').write_bytes(subprocess.check_output(['git','show',f'{FROZEN}:src/config.h'],cwd=ROOT))
old='''            change = clampf(ki*vel_err,-c.recovery_rate,c.recovery_rate)*dt'''
new='''            if self.ramp_complete and vel_err*self.vel_sp_integral<0:
                ki *= UNWIND_MULT
            change = clampf(ki*vel_err,-c.recovery_rate,c.recovery_rate)*dt'''
assert source.count(old)==1
source=source.replace(old,new)
module_path=frozen_root/'scripts/settling-unwind-sim.py';module_path.write_text(source)
spec=importlib.util.spec_from_file_location('settling_unwind_sim',module_path);s=importlib.util.module_from_spec(spec);sys.modules[spec.name]=s;spec.loader.exec_module(s)
baseclass=s.OuterController
last=None
class Observed(baseclass):
 def __init__(self,*a,**kw):
  global last
  super().__init__(*a,**kw);last=self;self.first_settle=None
 def tick(self,*a,**kw):
  was=self.startup_active
  super().tick(*a,**kw)
  if was and not self.startup_active and self.first_settle is None:self.first_settle=a[0]/1000
s.OuterController=Observed
cfg=s.current_firmware_config();plant=s.load_fitted_params(ROOT/'evidence/balance-review/model-fit-speed.json')
broad='--broad' in sys.argv
results=[]
for mult in ((1.,1.5,2.) if broad else (1.,1.5,2.,3.)):
 s.UNWIND_MULT=mult;cases=[]
 scenarios=(itertools.product((-1.9,-3.5,-4.5),(.7,1.,1.4),(.7,1.,1.4),(.01,.03,.06),(-.8,.8),(1,2))
            if broad else ((d,a,1.,t,o,1) for d,a,t,o in itertools.product((-1.9,-3.5,-4.5),(.7,1.4),(.01,.06),(-.4,.4))))
 for delta,af,bf,tau,offset,seed in scenarios:
  p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1.,fb_hold_ticks=1,vel_meas_noise=.1)
  r=s.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed)
  early=r.fell and (r.ramp_complete_s is None or r.t[-1]<r.ramp_complete_s+3)
  cases.append(dict(delta=delta,af=af,bf=bf,tau=tau,offset=offset,seed=seed,fell=r.fell,early=early,late=r.fell and not early,settle=last.first_settle,peak=abs(r.peak_drift_standup)))
 valid=[c['settle'] for c in cases if not c['fell'] and c['settle'] is not None]
 summary=dict(unwind_multiplier=mult,early=sum(c['early'] for c in cases),late=sum(c['late'] for c in cases),survived_and_settled=len(valid),median_settle=statistics.median(valid) if valid else None)
 if results:
  summary['new_failures']=sum(not a['fell'] and b['fell'] for a,b in zip(results[0]['cases'],cases))
  summary['rescues']=sum(a['fell'] and not b['fell'] for a,b in zip(results[0]['cases'],cases))
 if results:
  matched=[b['settle']-a['settle'] for a,b in zip(results[0]['cases'],cases) if not a['fell'] and not b['fell'] and a['settle'] is not None and b['settle'] is not None]
  summary['paired_settle_cases']=len(matched)
  summary['paired_median_settle_delta']=statistics.median(matched) if matched else None
 print(summary,flush=True);results.append(dict(summary=summary,cases=cases))
(OUT/('unwind-broad.json' if broad else 'unwind-screen.json')).write_text(json.dumps(results,indent=2)+'\n')
