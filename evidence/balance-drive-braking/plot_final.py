from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path(__file__).parent
x=json.loads((out/'final-nominal.json').read_text())
fig,axs=plt.subplots(3,1,figsize=(10,7.8),sharex=True,layout='constrained')
for label,color,title in [('v4','#777777','Installed v4'),('candidate','#007c91','Progressive braking')]:
 r=x['full_'+label];t=np.array(r['t'])-9;mask=(t>=-1)&(t<=9)
 v=np.array(r['v']);target=np.array(r['target']);pos=np.array(r['drift']);zero=np.argmin(abs(t));roll=np.array(r['roll'])
 axs[0].plot(t[mask],v[mask],color=color,label=title)
 axs[0].plot(t[mask],target[mask],color=color,linestyle='--',alpha=.7)
 axs[1].plot(t[mask],(pos-pos[zero])[mask],color=color)
 axs[2].plot(t[mask],(roll-np.median(roll[(t<-2)&(t>-4)]))[mask],color=color)
for ax in axs:
 ax.axvline(0,color='#c1573e',lw=1);ax.axhline(0,color='#aaaaaa',lw=.5);ax.grid(alpha=.2)
axs[0].legend(loc='upper right');axs[0].set_ylabel('Wheel speed (rad/s)')
axs[0].set_title('Illustrative model stop — solid: motion; dashed: requested speed',loc='left')
axs[1].set_ylabel('Net travel after center\n(wheel radians)')
axs[2].set_ylabel('Body angle change (°)');axs[2].set_xlabel('Seconds since CH2 centers')
fig.suptitle('Braking v5: faster reference ramp, gentle finish',fontsize=15)
fig.savefig(out/'final-model-stop.png',dpi=150)
