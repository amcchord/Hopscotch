import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('telemetry_logs/bal_20260914_223915_capture-trim-assisted.csv')
lines=p.read_text().splitlines();start=next(i for i,x in enumerate(lines) if x.startswith('t_ms,'))
r=[x for x in csv.DictReader(lines[start:]) if x.get('state')=='2']
t0=float(r[0]['t_ms']);t=[(float(x['t_ms'])-t0)/1000 for x in r]
f,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,layout='constrained')
for ax,col,label in zip(axes,('filtered_vel','meas_drift','vel_integral'),('Wheel speed (rad/s)','Wheel travel (rad)','Learned correction (degrees)')):
 ax.plot(t,[float(x[col]) for x in r],color='#2166ac',lw=2)
 ax.axvline(.72,color='#0a8a60',ls='--',lw=1.8,label='New detector replay: 0.72 s')
 ax.axvline(2.961,color='#b45309',ls=':',lw=1.8,label='Old learning enabled: 2.96 s')
 ax.set_ylabel(label);ax.grid(alpha=.2);ax.set_xlim(0,4)
axes[0].legend(loc='upper left',frameon=False)
axes[-1].set_xlabel('Seconds after balance engagement')
f.suptitle('Last physical trial: the new detector catches the onset much earlier\nRecorded old-firmware motion; this is detection replay, not a new balance result',fontsize=13)
f.savefig('evidence/balance-startup-recovery/detection-replay.png',dpi=160)
