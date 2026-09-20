"""Normal verified deployment, also testing transmitter-on OTA with a 4s gap.

Use the unchanged production updater; inject the gap only within its sender.
No bypass of manifest, saved-log backup, maintenance or postflight checks.
"""
import json
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import robot_wifi as w

host = 'http://192.168.1.172'
secrets = Path('/Users/austinmcchord/Development/Hopscotch/src/network_secrets.h')
package = ROOT / 'artifacts/lowering-v4-fast/candidate'
directory = ROOT / 'output/lowering-v4-fast-deployment'
sender = w.upload_application
real_sleep = time.sleep

def checked_sender(*args, **kwargs):
    preflight = json.loads(w.request(host, '/api/telemetry', timeout=5))
    w.require_idle(preflight)
    if not preflight['link_up'] or preflight['rc_age_ms'] > 100:
        raise ValueError('Transmitter-on acceptance requires a fresh live RC link')
    stop = threading.Event()
    snapshots = []
    count = 0
    def monitor():
        while not stop.is_set():
            try:
                state = json.loads(w.request(host, '/api/telemetry', timeout=3))
                snapshots.append({k:state.get(k) for k in ('uptime_ms','age_ms','link_up','rc_age_ms',
                     'drive_armed','arm_armed','maintenance','ota')})
            except Exception as exc:
                snapshots.append({'request_error':str(exc)})
            stop.wait(2)
    def paced_gap(seconds):
        nonlocal count
        count += 1
        if count == 256:
            print('Pausing sender for four seconds after 256 KiB', flush=True)
            real_sleep(4)
        real_sleep(seconds)
    thread = threading.Thread(target=monitor, daemon=True)
    thread.start()
    try:
        with patch.object(w.time, 'sleep', paced_gap):
            result = sender(*args, **kwargs)
    finally:
        stop.set()
        thread.join(timeout=5)
    result.update(transmitter_on_preflight=True, rc_snapshots=snapshots,
                  gap_injected=count>=256, gap_seconds=4, gap_after_body_bytes=262144)
    return result

with patch.object(w, 'upload_application', checked_sender):
    record = w.deploy(host, package/'firmware.bin', package/'manifest.json', directory, secrets)

samples = [x for x in record['transfer']['rc_snapshots'] if 'uptime_ms' in x]
final = record['final_state']
acceptance = dict(
    transmitter_on_during_upload=bool(samples) and all(x['link_up'] and x['rc_age_ms']<=100 for x in samples),
    motors_disarmed_throughout=all(not x['drive_armed'] and not x['arm_armed'] for x in samples),
    transmitter_on_after_reboot=final['link_up'] and final['rc_age_ms']<=100,
    receive_gap_survived=record['transfer']['gap_injected'] and record['status']=='installed_verified',
    saved_run_unchanged=record['saved_run_before']==record['saved_run_after'],
    snapshot_count=len(samples))
acceptance['passed'] = all(v for k,v in acceptance.items() if k!='snapshot_count')
record['transmitter_on_acceptance'] = acceptance
(directory/'deployment.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(acceptance), flush=True)
