from experiment_model import *
import itertools,json,csv,bisect
with (ROOT/'telemetry_logs/bal_20260919_wifi_v4_slow_stop.csv').open() as f: rr=list(csv.DictReader(line for line in f if not line.startswith('#')))
bal=[r for r in rr if r['state']=='2'];t0=float(bal[0]['t_ms']);times=[(float(r['t_ms'])-t0)/1000 for r in bal]
def raw(x):
 x=float(x);return 0 if x==0 else np.copysign(abs(x)*.94+.06,x)
def replay(t):
 r=bal[min(len(bal)-1,max(0,bisect.bisect_right(times,t)-1))]
 return raw(r['pilot_forward']) if t<times[-1] else 0,raw(r['pilot_steering']) if t<times[-1] else 0,True
rows=[]
for A,B,wn,z,delay in itertools.product((8,25,45),(4,7,13,16),(40,60),(.35,.6),(0,.01)):
 p=replace(plant,A=A,B=B,motor_omega=wn,motor_zeta=z,motor_delay_s=delay,sensor_delay_s=.005)
 for profile in ('push','replay'):
  fn=replay if profile=='replay' else profiles['small']
  pushes=[sim.Push(at_s=4,delta_rate=10,delta_vel=.5),sim.Push(at_s=15,delta_rate=-10,delta_vel=-.5)] if profile=='push' else []
  pair={}
  for label,dec in [('v4',8),('arm_capped20',20)]:
   r,pp=run([8.8,1.5,3,.006,6,dec,.3,.12,1],fn,p,duration=58 if profile=='replay' else 36,pushes=pushes,brake=(0,0,1,4,.08),arm_cap=float(np.float32(.008333333)*np.float32(8)))
   pair[label]=metrics(r,pp)
  rows.append(dict(A=A,B=B,wn=wn,z=z,delay=delay,profile=profile,**pair))
summary={}
for label in pair:
 summary[label]={pr:dict(falls=sum(r[label]['fell'] for r in rows if r['profile']==pr),new_falls=sum(r[label]['fell'] and not r['v4']['fell'] for r in rows if r['profile']==pr)) for pr in ('push','replay')}
print(json.dumps(summary,indent=2),flush=True)
(OUT/'arm-capped-disturbance.json').write_text(json.dumps(dict(summary=summary,cases=rows),indent=2)+'\n')
