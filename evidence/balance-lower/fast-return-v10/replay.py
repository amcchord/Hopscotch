#!/usr/bin/env python3
"""Check normal-mode preservation and locate fast-mode command divergence."""
import csv
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from simulate_lowering import Policy


def replay(policy, rows):
    forward=[rows[0]['arm_l_tgt'],rows[0]['arm_r_tgt']]
    rows=[r for r in rows if (int(r['pilot_flags'])>>8)&15]
    handle=policy.lib.lower_new();out=(C.c_float*7)();commands=[]
    try:
        for n,r in enumerate(rows):
            values=(C.c_float*11)(r['roll'],r['roll_rate'],r['angle_err'],r['bl_vel'],r['br_vel'],
                r['arm_l']-forward[0],r['arm_r']-forward[1],r['arm_l_vel'],r['arm_r_vel'],r['arm_l_torque'],r['arm_r_torque'])
            if not n:
                assert policy.lib.lower_request_mode(handle,int(r['t_ms']),values,True,1.768,-1.767,policy.fast)
            policy.lib.lower_step(handle,int(r['t_ms']),r['sample_dt_ms']/1000,values,True,out)
            commands.append([r['t_ms'],*out,policy.lib.lower_arm_speed(handle)])
            if int(out[2]) in (7,8):break
        return commands,policy.lib.lower_reason(handle).decode()
    finally:policy.lib.lower_delete(handle)


def main():
    source='b9763c2f5e2d66dcc28da4b1bc568dac29751b17'
    folder=ROOT/'output/fast-lower-v10-recorded-baseline';folder.mkdir(parents=True,exist_ok=True)
    (folder/'balance_lower.h').write_bytes(subprocess.check_output(['git','show',source+':src/balance_lower.h'],cwd=ROOT))
    old,normal,fast=Policy(folder),Policy(),Policy(fast=True)
    results=[]
    for name in ('v7_success','v8_stop','v9_success'):
        path=ROOT/'telemetry_logs'/('bal_20260920_forward_catch_'+name+'_wifi.csv')
        rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(x for x in path.read_text().splitlines() if not x.startswith('#'))]
        a,ar=replay(old,rows);b,br=replay(normal,rows);c,cr=replay(fast,rows)
        assert a==b and ar==br
        divergence=next((x[0] for x,y in zip(b,c) if x!=y),None)
        support=next(x[0] for x in b if int(x[3])==3)
        assert divergence is not None and divergence>support
        results.append(dict(capture=str(path.relative_to(ROOT)),normal_commands_identical=True,
            support_ms=support,fast_first_command_divergence_ms=divergence,
            normal_replay_end=br,fast_fixed_sensor_replay_end=cr))
    result=dict(baseline_source=source,header_sha256=hashlib.sha256((ROOT/'src/balance_lower.h').read_bytes()).hexdigest(),
        runs=results,limitation='Old recorded sensors cannot predict the physical trajectory after fast commands diverge. This verifies normal equivalence and unchanged preparation/catch, not fast hardware acceptance.')
    (Path(__file__).parent/'replay.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
