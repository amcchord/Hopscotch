#!/usr/bin/env python3
"""Reproducible inventory of every historical CSV, without inferring fall times."""
import csv
import hashlib
import json
from pathlib import Path
from analyze_balance_logs import load_rows, parse_config, as_float, as_int
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/balance-review'
records = []
for path in sorted((ROOT/'telemetry_logs').glob('*.csv')):
    rows = load_rows(path)
    config = parse_config(path)
    balance = [r for r in rows if r.get('state') == '2']
    data = dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                rows=len(rows), balance_rows=len(balance), motor_mode=config.get('motor_mode','legacy/unspecified'),
                end_reason=config.get('end_reason','unknown'),
                possible_capacity_cutoff=len(rows)>=2999 and 'end_reason' not in config)
    if balance:
        elapsed = (as_float(balance[-1],'t_ms')-as_float(balance[0],'t_ms'))/1000
        ramp = [r for r in balance if as_int(r,'flags') & 0x40]
        pre = [r for r in balance if not (as_int(r,'flags') & 0x40)]
        data.update(balance_elapsed_s=elapsed,
                    max_balance_gap_ms=max((as_float(b,'t_ms')-as_float(a,'t_ms') for a,b in zip(balance,balance[1:])), default=0),
                    measured_drift_peak_rad=max((abs(as_float(r,'meas_drift')) for r in balance), default=0),
                    tracking_within_2deg_percent=sum(abs(as_float(r,'roll')-as_float(r,'setpoint'))<2 for r in balance)*100/len(balance))
        if ramp:
            data['ramp_complete_s_after_engage']=(as_float(ramp[0],'t_ms')-as_float(balance[0],'t_ms'))/1000
            data['drift_at_ramp_rad']=as_float(ramp[0],'meas_drift')
        if pre and 'sp_offset' in pre[0]:
            peak=max(pre,key=lambda r:abs(as_float(r,'setpoint')-as_float(r,'roll')))
            data['pre_ramp_peak_tracking_error_deg']=as_float(peak,'setpoint')-as_float(peak,'roll')
            data['pre_ramp_offset_at_peak_error_deg']=as_float(peak,'sp_offset')
        if ramp and 'sp_offset' in ramp[0]:
            after = [r for r in ramp if as_float(r,'t_ms') <= as_float(ramp[0],'t_ms') + 1500]
            peak = max(after, key=lambda r:abs(as_float(r,'setpoint')-as_float(r,'roll')))
            data['post_ramp_peak_tracking_error_deg'] = as_float(peak,'setpoint')-as_float(peak,'roll')
            data['post_ramp_offset_at_peak_error_deg'] = as_float(peak,'sp_offset')
            data['post_ramp_peak_s_after_engage'] = (as_float(peak,'t_ms')-as_float(balance[0],'t_ms'))/1000
    records.append(data)
OUT.mkdir(parents=True,exist_ok=True)
# CSV allows missing old-schema measurements to remain explicit NaN/empty.
columns=list(dict.fromkeys(k for r in records for k in r))
with (OUT/'historical-inventory.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows(records)
summary=dict(files=len(records),unique_sha256=len(set(r['sha256'] for r in records)),
             total_rows=sum(r['rows'] for r in records),speed_mode_files=sum(r['motor_mode']=='speed' for r in records),
             possible_capacity_cutoffs=sum(r['possible_capacity_cutoff'] for r in records))
print(json.dumps(summary,indent=2))
for r in records:
    if r['file'] in ('bal_20260703_231458.csv','bal_20260703_233710.csv','bal_20260412_194229.csv','bal_20260409_220324.csv'):
        print(json.dumps(r,indent=2))
