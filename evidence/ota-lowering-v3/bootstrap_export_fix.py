"""One-time application-only recovery from the documented v2 export watchdog.

No pre-update GET /api/log: it reset the installed image on the saved run.
Do not make this the routine updater. Preserve filesystem and retrieve after.
"""
import json
from pathlib import Path
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import robot_wifi as w

host = 'http://192.168.1.172'
secrets = Path('/Users/austinmcchord/Development/Hopscotch/src/network_secrets.h')
package = ROOT / 'artifacts/lowering-v3-export/candidate'
directory = ROOT / 'output/lowering-v3-export-recovery'
directory.mkdir(exist_ok=False)
manifest = json.loads((package / 'manifest.json').read_text())
image = (package / 'firmware.bin').read_bytes()
identity = w.application_identity(image, manifest)
record = dict(started_utc=datetime.now(timezone.utc).isoformat(), status='preflight',
              source_commit=manifest['source_commit'], application=identity,
              motion_initiated=False, filesystem_written=False,
              backup_exception='Installed v2 GET /api/log timed out with task-watchdog reset before upload. '
              'Latest run not backed up before this recovery. Application-only update preserves filesystem; '
              'retrieve and validate immediately afterward. Exact old application retained for recovery.',
              previous_attempt='evidence/ota-lowering-v3/preflight-export-failure.json')

def save():
    (directory / 'deployment.json').write_text(json.dumps(record, indent=2) + '\n')

try:
    before = json.loads(w.request(host, '/api/info', timeout=5))
    state = json.loads(w.request(host, '/api/telemetry', timeout=5))
    record.update(preflight_info=before, preflight_state=state)
    save()
    assert before['image_sha256'] == '7668a0215df34b7e5c0030a23705ba8016343260d1e6bab7aacfe202897c7190'
    assert before['running_slot'] == 'app0' and len(image) <= before['ota_capacity']
    powered = [m['id'] for m in state['motors'] if m['online']]
    w.require_idle(state, powered)
    w.device_token(secrets)
    print('Uploading application-only export recovery + v3 lowering + OTA transport', flush=True)
    record['transfer'] = w.upload_application(host, image, identity, secrets)
    save()
    print(json.dumps(record['transfer']), flush=True)
    info, state = w.wait_for_image(host, identity['esp_image_digest'], before['running_slot'], powered)
    assert info.get('ota_transport_version') == 2
    record.update(postflight_info=info, postflight_state=state, status='installed_log_retrieval_pending')
    save()
    print('Exact new image verified; retrieving previously inaccessible saved run', flush=True)
    record['recovered_run'] = w.archive_log(host, directory, 'recovered', secrets)
    final_info = json.loads(w.request(host, '/api/info', timeout=5))
    final_state = json.loads(w.request(host, '/api/telemetry', timeout=5))
    w.require_idle(final_state, powered)
    assert final_info['image_sha256'] == identity['esp_image_digest']
    assert final_state['uptime_ms'] >= state['uptime_ms']
    record.update(final_info=final_info, final_state=final_state, status='installed_and_saved_run_recovered',
                  completed_utc=datetime.now(timezone.utc).isoformat())
    save()
    print(json.dumps({'status': record['status'], 'saved_run': record['recovered_run']}), flush=True)
except Exception as exc:
    record.update(status='verification_failed', error=str(exc))
    save()
    raise
