#!/usr/bin/env python3
"""Render measured failure versus the explicitly hypothetical candidate trace."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
old=list(csv.DictReader(x for x in (ROOT/'telemetry_logs/bal_20260920_lowering_trial_wifi.csv').read_text().splitlines() if not x.startswith('#')))
old=[r for r in old if float(r['t_ms'])>=37020]
new=list(csv.DictReader((HERE/'nominal.csv').read_text().splitlines()))
t=[float(r['time_s']) for r in new]
fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True,constrained_layout=True)
fig.suptitle('CH11: prepare → initiate forward fall → arm catch → lower\nCandidate v2 is simulation; the dashed red trace is the failed physical v1 trial',fontsize=13)
axes[0].plot(t,[float(r['tilt_deg']) for r in new],label='Candidate model',color='#19647e')
axes[0].plot([(float(r['t_ms'])-37020)/1000 for r in old],[float(r['roll']) for r in old],label='Measured failed v1',color='#b44b40',ls='--')
axes[0].axhline(90,color='#888',lw=.7);axes[0].set_ylabel('Tilt (deg)\n↑ backward');axes[0].legend()
axes[1].plot(t,[float(r['rate_dps']) for r in new],color='#19647e');axes[1].axhline(0,color='#888',lw=.7);axes[1].set_ylabel('Model body rate\n(deg/s)')
axes[2].plot(t,[-float(r['arm_l']) for r in new],label='Left measured')
axes[2].plot(t,[-float(r['target_l']) for r in new],label='Left target',ls='--')
axes[2].plot(t,[float(r['arm_r']) for r in new],label='Right measured')
axes[2].set_ylabel('Model forward\narm reach (rad)');axes[2].legend(loc='lower left',ncol=3,fontsize=9)
axes[3].plot(t,[float(r['wheel_rad_s']) for r in new],label='Rear speed')
axes[3].plot(t,[float(r['wheel_request']) if int(float(r['committed'])) else float('nan') for r in new],label='Fall/catch command',ls='--')
axes[3].set_ylabel('Model rear wheels\n(rad/s)');axes[3].set_xlabel('Seconds after CH11 request');axes[3].legend(loc='lower right',fontsize=9)
phases={2:'prepare',9:'forward initiation',10:'catch',3:'supported descent',4:'flat hold',5:'retract'}
prev=None
for r in new:
 phase=int(float(r['phase']))
 if phase!=prev and phase in phases:
  x=float(r['time_s'])
  for ax in axes:ax.axvline(x,color='#777',lw=.6,ls=':')
  axes[0].annotate(phases[phase],(x,108),xytext=(3,0),textcoords='offset points',rotation=90,va='top',fontsize=8)
 prev=phase
for ax in axes:ax.grid(alpha=.2)
axes[0].set_ylim(-5,114)
fig.savefig(HERE/'comparison.png',dpi=160)
