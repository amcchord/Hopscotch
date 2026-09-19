"""Observed closed-loop frequency ratios, not a causal plant identification."""
import csv,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
path=ROOT/'telemetry_logs/bal_20260919_185559_drive-agility-wobble-wall.csv'
rows=list(csv.DictReader(l for l in path.read_text().splitlines() if not l.startswith('#')))
a={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
t=(a['t_ms']-a['t_ms'][a['state']==2][0])/1000
v=(a['bl_vel']+a['br_vel'])/2
results=[]
for start,end in [(7,9),(9.5,12.5),(14,16),(17.5,20.5),(20.5,23),(24,27),(27,30)]:
 mask=(t>=start)&(t<end);n=mask.sum();freq=np.fft.rfftfreq(n,.02)
 def ft(x):return np.fft.rfft((x[mask]-np.mean(x[mask]))*np.hanning(n))
 r=ft(a['gyro_raw']);band=(freq>=3)&(freq<=6);i=np.flatnonzero(band)[np.argmax(abs(r[band]))]
 def ratio(num,den):
  z=ft(num)[i]/ft(den)[i];return dict(magnitude=float(abs(z)),phase_deg=float(np.angle(z,deg=True)))
 results.append(dict(start_s=start,end_s=end,frequency_hz=float(freq[i]),raw_gyro_over_wheel=ratio(a['gyro_raw'],v),wheel_over_command=ratio(v,a['motor_vel']),filtered_over_raw_gyro=ratio(a['roll_rate'],a['gyro_raw'])))
(OUT/'observed-frequency-ratios.json').write_text(json.dumps(dict(source=str(path.relative_to(ROOT)),method='Hann-window FFT peak of raw gyro in 3–6 Hz; observed closed-loop ratios; no wall/contact interval >=32s included',windows=results),indent=2)+'\n')
print(json.dumps(results,indent=2))
