"""First braking screen: only change requested deceleration; no hardware I/O."""
import sys,json,itertools
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'balance-drive-damping'))
from model import *
OUT=Path(__file__).parent
rows=[]
# Stand, drive to requested 10 rad/s, center at 6 s; both directions.
for A,B,wn,z,delay,sign in itertools.product((8,25,45),(4,7,13,16),(40,60),(.35,.6),(0,.01),(-1,1)):
 p=replace(plant,A=A,B=B,motor_omega=wn,motor_zeta=z,motor_delay_s=delay,sensor_delay_s=.005)
 profile=lambda t:(sign*.53 if 2<=t<6 else 0,0,True)
 pair={}
 for dec in (8,16,20,24):
  params=[8.8,1.5,3,.006,6,dec,.3,.12,1]
  r,pp=run(params,profile,p,duration=18);t=np.array(r.t);v=np.array(r.wheel_vel);pos=np.array(r.drift)
  ii=np.flatnonzero((t>=6)&(abs(v)<.3));calm=None
  for i in ii:
   if i+20<len(v) and max(abs(v[i:i+21]))<.3:calm=float(t[i]-6);break
  start=int(np.argmin(abs(t-6)));end=int(np.argmin(abs(t-12)))
  pair[str(dec)]=dict(fell=r.fell,commanded=bool(max(abs(np.array(pp.rows)[:,5]))>.1),first_stop=float(t[ii[0]]-6) if len(ii) else None,calm_stop=calm,peak_reverse=float(max(0,max(-sign*v[start:]))),travel=float(abs(pos[end]-pos[start])),tilt_span=float(np.ptp(r.roll)))
 rows.append(dict(A=A,B=B,wn=wn,z=z,delay=delay,sign=sign,**pair))
summary={}
for dec in (8,16,20,24):
 d=str(dec);cohort=[r for r in rows if r[d]['commanded'] and r['8']['commanded'] and not r[d]['fell'] and not r['8']['fell']]
 summary[d]=dict(falls=sum(r[d]['fell'] for r in rows),new_falls=sum(r[d]['fell'] and not r['8']['fell'] for r in rows),commanded=sum(r[d]['commanded'] for r in rows),median_stop=float(np.median([r[d]['first_stop'] for r in cohort if r[d]['first_stop'] is not None])),median_reverse=float(np.median([r[d]['peak_reverse'] for r in cohort])),median_travel=float(np.median([r[d]['travel'] for r in cohort])))
(OUT/'ramps.json').write_text(json.dumps(dict(summary=summary,cases=rows),indent=2)+'\n');print(json.dumps(summary,indent=2))
