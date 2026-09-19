import sys,json
sys.path.insert(0,'evidence/balance-drive-damping')
from model import *
p=replace(plant,motor_omega=50,motor_zeta=.35,motor_delay_s=.01,sensor_delay_s=.005)
rows=[]
for kv in (3,4,4.5):
 for decel in (8,12,16):
  pars=[8.8,1.5,kv,.006,6,decel,.3,.12,1]
  r,pp=run(pars,'full',p);t=np.array(r.t);v=np.array(r.wheel_vel);pos=np.array(r.drift)
  later=(t>=9)&(t<18);idx=np.flatnonzero(later&(abs(v)<.3))
  rows.append(dict(params=pars,**metrics(r,pp),first_stop=float(t[idx[0]]-9) if len(idx) else None,travel_after_center=float(pos[(t>=9)&(t<18)][-1]-pos[np.argmin(abs(t-9))])))
print(json.dumps(rows,indent=2));(OUT/'braking-tradeoff.json').write_text(json.dumps(rows,indent=2)+'\n')
