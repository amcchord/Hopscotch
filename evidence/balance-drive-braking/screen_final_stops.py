from frozen_baseline import *
import itertools,json
rows=[]
for A,B,wn,z,delay,sign in itertools.product((8,25,45),(4,7,13,16),(40,60),(.35,.6),(0,.01),(-1,1)):
 p=replace(plant,A=A,B=B,motor_omega=wn,motor_zeta=z,motor_delay_s=delay,sensor_delay_s=.005)
 profile=lambda t:(sign*.53 if 2<=t<6 else 0,0,True)
 # Frozen installed helper library and matching API are initialized by final_screen.
 pair={}
 for label,api in [('v4',oldlib),('candidate',lib)]:
  pp=Pilot(None,profile,api=api)
  r=sim.simulate(cfg,p,engage_offset_deg=0,duration_s=18,seed=1,pilot_factory=lambda:pp,settled_start=True)
  t=np.array(r.t);v=np.array(r.wheel_vel);pos=np.array(r.drift)
  ii=np.flatnonzero((t>=6)&(abs(v)<.3));calm=None
  for i in ii:
   if i+20<len(v) and max(abs(v[i:i+21]))<.3:calm=float(t[i]-6);break
  start=int(np.argmin(abs(t-6)));end=int(np.argmin(abs(t-12)))
  pair[label]=dict(fell=r.fell,commanded=bool(max(abs(np.array(pp.rows)[:,5]))>.1),first_stop=float(t[ii[0]]-6) if len(ii) else None,calm_stop=calm,peak_reverse=float(max(0,max(-sign*v[start:]))),travel=float(abs(pos[end]-pos[start])),tilt_span=float(np.ptp(r.roll)))
 rows.append(dict(A=A,B=B,wn=wn,z=z,delay=delay,sign=sign,**pair))
summary={}
c=[r for r in rows if all(r[k]['commanded'] and not r[k]['fell'] for k in ('v4','candidate'))]
for label in pair:
 summary[label]=dict(falls=sum(r[label]['fell'] for r in rows),new_falls=sum(r[label]['fell'] and not r['v4']['fell'] for r in rows),common_cohort=len(c),median_stop=float(np.median([r[label]['first_stop'] for r in c if r[label]['first_stop'] is not None])),median_calm=float(np.median([r[label]['calm_stop'] for r in c if r[label]['calm_stop'] is not None])),calm_count=sum(r[label]['calm_stop'] is not None for r in c),median_reverse=float(np.median([r[label]['peak_reverse'] for r in c])),median_travel=float(np.median([r[label]['travel'] for r in c])))
print(json.dumps(summary,indent=2),flush=True)
(OUT/'final-stops.json').write_text(json.dumps(dict(summary=summary,cases=rows),indent=2)+'\n')
