import sys,json,time
from pathlib import Path
from dataclasses import replace,asdict
import statistics
sys.path.insert(0,str(Path.cwd()/'scripts'))
import balance_sim as s
base=s.current_firmware_config()

plant=s.load_fitted_params(Path('evidence/balance-review/model-fit-speed.json'))
variants={'baseline':base,'no_ramp_damper':replace(base,no_ramp_damper=True),'offset_05':replace(base,ramp_off_clamp=.5),'return_10':replace(base,arm_return_speed=1.),'return_08_no_damper':replace(base,arm_return_speed=.8,no_ramp_damper=True),'lead075':replace(base,carrot_effective=True,carrot_eff_lead_deg=.75),'lead15':replace(base,carrot_effective=True,carrot_eff_lead_deg=1.5),'eq_tracker':replace(base,eq_track=True),'gate2':replace(base,arm_return_gate=True,arm_gate_err_deg=2.,arm_gate_vel=3.)}
variants={'legacy_decode':base,'correct_decode':base,'kp24':replace(base,kp=2.4),'kp28':replace(base,kp=2.8),'smooth12':replace(base,arm_return_acceleration=12.),'rampkp10':replace(base,vel_sp_kp_low=1.),'off25':replace(base,ramp_off_clamp=2.5)}
results=[]
for name,cfg in variants.items():
    cases=[]
    for delta in (-1.9,-3.3,-4.5):
        for lag in (.06,.15):
            for offset in (-.8,.8):
                r=s.simulate(cfg,replace(plant,eq_tip_delta=delta,tau_m=lag,B=plant.B*33/50,feedback_velocity_scale=(33/50 if name=='legacy_decode' else 1)),engage_offset_deg=offset,duration_s=15,seed=1)
                cases.append({'delta':delta,'lag':lag,'offset':offset,'fell':r.fell,'duration':float(r.t[-1]),'ramp':r.ramp_complete_s,'drift_at_ramp':r.drift_at_ramp,'peak_drift':r.peak_drift_standup})
    good=[c for c in cases if not c['fell']]
    row={'name':name,'failures':sum(c['fell'] for c in cases),'median_peak_drift':statistics.median(abs(c['peak_drift']) for c in good) if good else None,'cases':cases}
    results.append(row);print({k:v for k,v in row.items() if k!='cases'},flush=True)
Path('evidence/balance-standup-fix/screening4.json').write_text(json.dumps(results,indent=2)+'\n')
