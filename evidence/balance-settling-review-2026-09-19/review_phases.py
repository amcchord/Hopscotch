"""Observed phase timing; no altered-controller replay or predicted motion."""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_balance_logs import load_rows
results=[]
for name in ('bal_20260914_233101_early-recovery-first-success.csv','bal_20260919_143953_early-recovery-repeat-success.csv'):
 rows=[r for r in load_rows(ROOT/'telemetry_logs'/name) if r['state']=='2']
 v=lambda k:np.array([float(r[k]) for r in rows]);t=(v('t_ms')-v('t_ms')[0])/1000
 peak=np.argmax(np.where(t<3,v('filtered_vel'),-100))
 zero=np.flatnonzero((t>t[peak])&(v('filtered_vel')<0))[0]
 reverse=np.argmin(np.where((t>t[zero])&(t<7),v('filtered_vel'),100))
 settle=np.flatnonzero(v('diag_flags').astype(int)&0x8000)[0]
 calm=(v('flags').astype(int)&64!=0)&(abs(v('filtered_vel'))<.7)&(abs(v('roll_rate'))<4)&(abs(v('angle_err'))<1)
 spans=[];start=None
 for i,ok in enumerate(calm):
  if ok and start is None:start=i
  if start is not None and (not ok or i==len(calm)-1):
   end=i-1 if not ok else i
   if t[start]<t[settle]:spans.append(dict(start_s=float(t[start]),last_s=float(t[end]),span_s=float(t[end]-t[start])))
   start=None
 results.append(dict(file=name,first_forward_stop_s=float(t[zero]),peak_reverse_speed_rad_s=float(v('filtered_vel')[reverse]),peak_reverse_time_s=float(t[reverse]),settle_s=float(t[settle]),stop_to_settle_s=float(t[settle]-t[zero]),integral_at_first_stop_deg=float(v('vel_integral')[zero]),integral_at_settle_deg=float(v('vel_integral')[settle]),approximate_calm_spans_before_settle=spans))
(Path(__file__).parent/'phases.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
