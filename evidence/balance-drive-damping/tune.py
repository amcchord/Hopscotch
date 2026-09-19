"""Record rejected/selected driving-only candidates against uncertain dynamics."""
from model import *
import itertools,json
candidates={
 'v3':OLD,
 'fast_rate15':[8.8,1.5,3,.006,6,8,.3,.12,1],
 'fast_rate20':[8.8,2,3,.006,6,8,.3,.12,1],
 'fast_rate10':[8.8,1,3,.006,6,8,.3,.12,1],
 'tau10':[8.8,1.5,3,.01,6,8,.3,.12,1],
 'lower_speed_gain':[10,1.2,2,.006,6,8,.3,.12,1],
}
rows=[]
for A,B,wn,z,delay in itertools.product((8,25,45),(10,13,16),(40,50),(.35,.6),(0,.01)):
 p=replace(plant,A=A,B=B,motor_omega=wn,motor_zeta=z,motor_delay_s=delay,sensor_delay_s=.005)
 for profile in ('small','full','loss'):
  pair={}
  for label,params in candidates.items():
   r,pilot=run(params,profile,p)
   m=metrics(r,pilot);t=np.array(r.t);v=np.array(r.wheel_vel)
   mask=(t>=3)&(t<min(t[-1],17))
   vv=v[mask];spec=np.fft.rfft((vv-np.mean(vv))*np.hanning(len(vv))) if len(vv)>20 else np.zeros(1)
   freq=np.fft.rfftfreq(len(vv),.02) if len(vv)>20 else np.zeros(1)
   m['ring_power']=float(np.sum(abs(spec[(freq>=3)&(freq<=6)])**2)/max(len(vv)**2,1))
   pair[label]=m
  rows.append(dict(A=A,B=B,wn=wn,z=z,delay=delay,profile=profile,**pair))
for name in candidates:
 print(name,dict(falls=sum(r[name]['fell'] for r in rows),new_falls=sum(r[name]['fell'] and not r['v3']['fell'] for r in rows),ring_median=float(np.median([r[name]['ring_power'] for r in rows])),ring_p95=float(np.quantile([r[name]['ring_power'] for r in rows],.95))),flush=True)
(OUT/'tuning-screen.json').write_text(json.dumps(dict(candidates=candidates,cases=rows),indent=2)+'\n')
