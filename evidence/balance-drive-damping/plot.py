"""Compare nominal offline trajectories; plot is not a physical prediction."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).parent;data=json.loads((OUT/'screen.json').read_text());n=data['nominal']
fig,axes=plt.subplots(3,2,figsize=(13,9),sharex='col',layout='constrained')
metrics={}
for col,profile in enumerate(('small','full')):
 for label,color in [('v3','#ea580c'),('candidate','#2563eb')]:
  d=n[profile+'_'+label];t=np.array(d['t']);v=np.array(d['v']);roll=np.array(d['roll']);target=np.array(d['target'])
  axes[0,col].plot(t,v,label=label,color=color,lw=1)
  axes[1,col].plot(t,roll,label=label,color=color,lw=1)
  axes[2,col].plot(t,d['drift'],label=label,color=color,lw=1)
  if label=='candidate':axes[0,col].plot(t,target,color='#64748b',ls='--',label='candidate requested',lw=1)
  moved=np.flatnonzero((t>=2)&(v>.5))
  stopped=np.flatnonzero((t>= (5 if profile=='small' else 9))&(np.abs(v)<.3))
  metrics[profile+'_'+label]=dict(first_positive_half_rad_after_input_s=float(t[moved[0]]-2) if len(moved) else None,first_below_point3_after_center_s=float(t[stopped[0]]-(5 if profile=='small' else 9)) if len(stopped) else None)
 axes[0,col].set_title(profile.capitalize()+' inputs, then center / reverse')
 axes[2,col].set_xlabel('Time from already-standing initial condition (seconds)')
for row,y in enumerate(('Wheel speed (rad/s)','Body angle (degrees)','Wheel travel (radians)')):
 for ax in axes[row]:ax.set_ylabel(y);ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8);ax.set_xlim(0,36)
fig.suptitle('Offline driving screen: revised feedback suppresses the fast ringing\nUncertain planar plant; quiet-hold ripple differs from hardware, yaw/traction/contact omitted')
fig.savefig(OUT/'model-comparison.png',dpi=150)
(OUT/'nominal-response.json').write_text(json.dumps(metrics,indent=2)+'\n');print(json.dumps(metrics,indent=2))
