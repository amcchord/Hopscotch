#!/usr/bin/env python3
"""Plot the measured July handoff, without treating log cutoff as a fall."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_balance_logs import load_rows, as_float, as_int

ROOT = Path(__file__).resolve().parents[1]
fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True, constrained_layout=True)
for col, run in enumerate(('231458', '233710')):
    rows = [r for r in load_rows(ROOT / f'telemetry_logs/bal_20260703_{run}.csv') if r.get('state') == '2']
    start = as_float(rows[0], 't_ms')
    rows = [r for r in rows if as_float(r,'t_ms') - start <= 9000]
    t = [(as_float(r,'t_ms')-start)/1000 for r in rows]
    ramp = next(ti for ti,r in zip(t,rows) if as_int(r,'flags') & 0x40)
    roll = [as_float(r,'roll') for r in rows]
    sp = [as_float(r,'setpoint') for r in rows]
    off = [as_float(r,'sp_offset') for r in rows]
    axes[0,col].plot(t,roll,label='Measured tilt',color='#245ca6')
    axes[0,col].plot(t,sp,label='Effective target',color='#d26925')
    axes[0,col].plot(t,[s-o for s,o in zip(sp,off)],label='Target minus outer offset',color='#488457',ls='--')
    axes[1,col].plot(t,off,color='#d26925',label='Outer offset')
    axes[1,col].axhline(1.5,color='#999999',ls=':',label='Candidate pre-ramp limit')
    axes[1,col].axhline(-1.5,color='#999999',ls=':')
    axes[2,col].plot(t,[as_float(r,'meas_drift') for r in rows],color='#245ca6',label='Wheel drift')
    for row in range(3):
        axes[row,col].axvline(ramp,color='#b22e42',ls='--',lw=1,label='Ramp complete' if row==2 else None)
        axes[row,col].grid(alpha=.17)
    axes[0,col].set_title(f'July 3, {run[:2]}:{run[2:4]}:{run[4:]} | ramp ends at {ramp:.2f}s')
    axes[2,col].set_xlabel('Seconds after balance engagement')
axes[0,0].set_ylabel('Degrees')
axes[1,0].set_ylabel('Outer target offset (deg)')
axes[2,0].set_ylabel('Wheel rotation from origin (rad)')
axes[0,0].legend(fontsize=8,loc='upper right')
axes[1,0].legend(fontsize=8)
axes[2,0].legend(fontsize=8)
fig.suptitle('The largest target surge follows arm-return ramp completion',fontsize=15)
fig.savefig(ROOT/'evidence/balance-review/july-handoff.png',dpi=180)
