#!/usr/bin/env python3
"""Hopscotch LAN telemetry, verified log download and application-only OTA."""
import argparse
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import re
import socket
import time
from datetime import datetime, timezone
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
UPLOAD_PROFILES = {
    'paced': dict(chunk_bytes=1024, interval=0.05),
    # TCP provides backpressure; no application-level sleep between sends.
    'fast': dict(chunk_bytes=16 * 1024, interval=0.0),
}
OTA_TIMING_HEADERS = {
    'elapsed_ms': 'X-OTA-Elapsed-Ms', 'write_us': 'X-OTA-Write-Us',
    'max_write_us': 'X-OTA-Max-Write-Us', 'max_receive_gap_ms': 'X-OTA-Max-Receive-Gap-Ms',
    'verify_us': 'X-OTA-Verify-Us',
}


def device_token(secrets_file=None):
    token = os.environ.get('HOPSCOTCH_API_TOKEN')
    if token:
        return token
    local = secrets_file or ROOT / 'src/network_secrets.h'
    if local.exists():
        match = re.search(r'^#define HOPSCOTCH_API_TOKEN "([^"\n]+)"', local.read_text(), re.M)
        if match:
            return match[1]
    raise ValueError('Set HOPSCOTCH_API_TOKEN or configure src/network_secrets.h')


def request(host, path, data=None, headers=None, auth=False, timeout=30, secrets_file=None):
    h = dict(headers or {})
    if auth:
        h['Authorization'] = 'Bearer ' + device_token(secrets_file)
    url = host.rstrip('/') + path
    return urllib.request.urlopen(urllib.request.Request(url, data=data, headers=h), timeout=timeout).read()


def application_identity(image, manifest=None):
    if len(image) < 1024 or image[0] != 0xE9:
        raise ValueError('Use an ESP application firmware.bin, not a filesystem or merged image')
    if image[23] != 1 or hashlib.sha256(image[:-32]).digest() != image[-32:]:
        raise ValueError('Application must include a valid ESP image SHA-256 trailer')
    identity = dict(application_bytes=len(image), application_sha256=hashlib.sha256(image).hexdigest(),
                    esp_image_digest=image[-32:].hex())
    if manifest is not None:
        for key, value in identity.items():
            if manifest.get(key) != value:
                raise ValueError(f'Frozen release manifest mismatch: {key}')
    return identity


def require_idle(state, powered_ids=()):
    """Client preflight; firmware's control-owner interlock is authoritative."""
    if state.get('maintenance_allowed') is not True or state.get('maintenance') is not False:
        raise ValueError('Disarm both groups and wait for maintenance/log saving to finish')
    for key in ('drive_armed', 'arm_armed', 'arming', 'saving_log', 'calibration', 'test_mode', 'simulation'):
        if state.get(key) is not False:
            raise ValueError(f'Robot is not ready: {key}')
    balance = state.get('balance', {})
    if (state.get('age_ms', 1000) > 250 or balance.get('state') != 'IDLE'
            or balance.get('active') is not False or balance.get('fault') != 0
            or balance.get('imu_age_us', 1000000) > 50000):
        raise ValueError('Fresh idle state and healthy IMU required')
    motors = state.get('motors', [])
    if len(motors) != 6 or any(m.get('enabled') is not False for m in motors):
        raise ValueError('All six motors must be disabled')
    for motor_id in powered_ids:
        motor = next((m for m in motors if m.get('id') == motor_id), {})
        if not motor.get('online') or motor.get('error') != 0 or motor.get('age_ms', 1000) > 500:
            raise ValueError(f'Powered motor {motor_id} has not returned healthy feedback')


def archive_log(host, directory, name, secrets_file=None):
    from validate_telemetry import validate
    raw = request(host, '/api/log', auth=True, timeout=120, secrets_file=secrets_file)
    # Preserve even a corrupt/incomplete export for diagnosis; never accept it as a backup.
    with (directory / f'{name}.wire').open('xb') as f:
        f.write(raw)
    if raw.strip() == b'[Balance] No log file found':
        return dict(samples=0, absent=True, wire_sha256=hashlib.sha256(raw).hexdigest())
    clean, count = validate(raw)
    with (directory / f'{name}.csv').open('xb') as f:
        f.write(clean)
    return dict(samples=count, csv_sha256=hashlib.sha256(clean).hexdigest(),
                wire_sha256=hashlib.sha256(raw).hexdigest())


def upload_application(host, image, identity, secrets_file=None, interval=0.05, chunk_bytes=1024):
    """Bounded multipart sends. Never retry after an ambiguous disconnect."""
    if not math.isfinite(interval) or not 0 <= interval <= 5 or not 1 <= chunk_bytes <= 65536:
        raise ValueError('Invalid upload pacing or chunk size')
    parsed = urllib.parse.urlsplit(host)
    if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.path not in ('', '/'):
        raise ValueError('Use an http(s) robot host without a path')
    connection_type = http.client.HTTPSConnection if parsed.scheme == 'https' else http.client.HTTPConnection
    conn = connection_type(parsed.hostname, parsed.port, timeout=120)
    boundary = 'hopscotch-' + os.urandom(12).hex()
    body = (f'--{boundary}\r\nContent-Disposition: form-data; name="firmware"; filename="firmware.bin"\r\n'
            'Content-Type: application/octet-stream\r\n\r\n').encode() + image + f'\r\n--{boundary}--\r\n'.encode()
    headers = {'Authorization': 'Bearer ' + device_token(secrets_file),
               'Content-Type': f'multipart/form-data; boundary={boundary}', 'Content-Length': str(len(body)),
               'X-Firmware-Size': str(len(image)), 'X-Firmware-SHA256': identity['application_sha256']}
    result = dict(chunk_bytes=chunk_bytes, interval_seconds=interval, timeout_seconds=120, tcp_nodelay=True)
    started = time.monotonic()
    sent = 0
    send_seconds = max_send_seconds = sleep_seconds = 0.0
    next_progress = 256 * 1024
    def capture_response(response):
        result.update(status=response.status, response=response.read().decode(errors='replace'))
        timings = {}
        for field, header in OTA_TIMING_HEADERS.items():
            value = response.getheader(header)
            if value is not None and re.fullmatch(r'[0-9]{1,20}', value):
                timings[field] = int(value)
        if timings:
            result['server_timings'] = timings
    try:
        conn.connect()
        conn.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.putrequest('POST', '/api/ota')
        for key, value in headers.items():
            conn.putheader(key, value)
        conn.endheaders()
        for offset in range(0, len(body), chunk_bytes):
            chunk = body[offset:offset + chunk_bytes]
            send_started = time.monotonic()
            try:
                conn.send(chunk)
            finally:
                duration = time.monotonic() - send_started
                send_seconds += duration
                max_send_seconds = max(max_send_seconds, duration)
            sent += len(chunk)
            if interval and sent < len(body):
                sleep_started = time.monotonic()
                time.sleep(interval)
                sleep_seconds += time.monotonic() - sleep_started
            if sent >= next_progress:
                print(f'Sent {sent}/{len(body)} bytes', flush=True)
                next_progress += 256 * 1024
        response_started = time.monotonic()
        try:
            response = conn.getresponse()
            capture_response(response)
        finally:
            result['response_wait_seconds'] = round(time.monotonic() - response_started, 6)
    except (OSError, http.client.HTTPException) as exc:
        result['transport_error'] = str(exc)
        # An early HTTP rejection may be readable even when the body send failed.
        try:
            response = conn.getresponse()
            capture_response(response)
        except (OSError, http.client.HTTPException):
            pass
    finally:
        conn.close()
        result.update(sent_bytes=sent, seconds=round(time.monotonic() - started, 3),
                      send_block_seconds=round(send_seconds, 6), max_send_block_seconds=round(max_send_seconds, 6),
                      pacing_sleep_seconds=round(sleep_seconds, 6))
    return result


def wait_for_image(host, expected, previous_slot=None, powered_ids=(), seconds=60):
    deadline = time.monotonic() + seconds
    last = 'Robot did not reconnect'
    while time.monotonic() < deadline:
        try:
            info = json.loads(request(host, '/api/info', timeout=2))
            state = json.loads(request(host, '/api/telemetry', timeout=2))
            if info.get('image_sha256') != expected:
                last = f"Running image is still {info.get('image_sha256', 'unknown')}"
            elif previous_slot is not None and info.get('running_slot') == previous_slot:
                last = 'Expected image reported without switching application slot'
            elif info.get('running_slot') not in ('app0', 'app1'):
                last = 'Running application slot is unavailable'
            else:
                require_idle(state, powered_ids)
                return info, state
        except (OSError, ValueError) as exc:
            last = str(exc)
        time.sleep(1)
    raise ValueError(f'Installed image/health not verified: {last}. Inspect before retrying or testing.')


def deploy(host, firmware, manifest_path=None, record_dir=None, secrets_file=None, upload_profile='paced'):
    if upload_profile not in UPLOAD_PROFILES:
        raise ValueError('Unknown upload profile')
    image = firmware.read_bytes()
    manifest = json.loads(manifest_path.read_text()) if manifest_path else None
    identity = application_identity(image, manifest)
    # Validate credentials before any maintenance request; never store them in evidence.
    device_token(secrets_file)
    directory = record_dir or ROOT / 'output' / datetime.now(timezone.utc).strftime('ota-%Y%m%dT%H%M%S.%fZ')
    directory.mkdir(parents=True, exist_ok=False)
    record = dict(started_utc=datetime.now(timezone.utc).isoformat(), application=identity,
                  source_commit=manifest.get('source_commit') if manifest else None, status='preflight',
                  motion_initiated=False, upload_profile=upload_profile)
    def save():
        (directory / 'deployment.json').write_text(json.dumps(record, indent=2) + '\n')
    save()
    print(f'Deployment record: {directory}', flush=True)
    try:
        before = json.loads(request(host, '/api/info', timeout=5))
        state = json.loads(request(host, '/api/telemetry', timeout=5))
        record.update(preflight_info=before, preflight_state=state)
        save()
        powered_ids = [m['id'] for m in state.get('motors', []) if m.get('online')]
        require_idle(state, powered_ids)
        if len(image) > before.get('ota_capacity', 0) or before.get('running_slot') not in ('app0', 'app1'):
            raise ValueError('Application does not fit or current OTA slot is unavailable')
        already_installed = before.get('image_sha256') == identity['esp_image_digest']
        print('Archiving saved run before upload', flush=True)
        record['saved_run_before'] = archive_log(host, directory, 'before', secrets_file)
        save()
        if already_installed:
            # A retry after a lost acknowledgment must not flash/reboot the same image again.
            info, state = wait_for_image(host, identity['esp_image_digest'], powered_ids=powered_ids)
            record.update(postflight_info=info, postflight_state=state, status='already_installed_verified')
        else:
            require_idle(json.loads(request(host, '/api/telemetry', timeout=5)), powered_ids)
            print(f'Uploading {len(image)} bytes with {upload_profile} profile; duration depends on link/flash', flush=True)
            record['transfer'] = upload_application(host, image, identity, secrets_file, **UPLOAD_PROFILES[upload_profile])
            save()
            print(json.dumps(record['transfer']), flush=True)
            # A lost HTTP response can still mean a successful installation. Read before any retry.
            info, state = wait_for_image(host, identity['esp_image_digest'], before['running_slot'], powered_ids)
            record.update(postflight_info=info, postflight_state=state)
            record['saved_run_after'] = archive_log(host, directory, 'after', secrets_file)
            if record['saved_run_before'] != record['saved_run_after']:
                raise ValueError('Saved run changed across OTA; both exports retained for inspection')
            record['status'] = 'installed_verified'
        record['final_state'] = json.loads(request(host, '/api/telemetry', timeout=5))
        require_idle(record['final_state'], powered_ids)
        record['completed_utc'] = datetime.now(timezone.utc).isoformat()
        save()
        print(f"Verified {record['status']}: slot {info['running_slot']}, build {info['build']}; "
              'image matches, both groups disarmed, saved run archived.', flush=True)
        return record
    except Exception as exc:
        record.update(status='verification_failed', error=str(exc))
        save()
        raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='http://hopscotch.local')
    p.add_argument('--secrets-file', type=Path, help='Read an existing private header in place (do not copy it)')
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    sub.add_parser('disarm')
    sub.add_parser('reconnect')
    log = sub.add_parser('log')
    log.add_argument('--output', type=Path)
    ota = sub.add_parser('ota')
    ota.add_argument('firmware', type=Path)
    ota.add_argument('--manifest', type=Path, help='Verify size and both digests against the frozen release manifest')
    ota.add_argument('--record-dir', type=Path, help='New directory for pre/post state and validated saved-run exports')
    ota.add_argument('--upload-profile', choices=UPLOAD_PROFILES, default='paced',
                     help='paced: proven 1 KiB/50 ms; fast: 16 KiB sends without artificial sleeps (hardware trial pending)')
    a = p.parse_args()
    if a.command == 'status':
        print(json.dumps(json.loads(request(a.host, '/api/telemetry')), indent=2))
    elif a.command == 'disarm':
        print(request(a.host, '/api/disarm', b'', auth=True, secrets_file=a.secrets_file).decode())
    elif a.command == 'reconnect':
        print(request(a.host, '/api/wifi/reconnect', b'', auth=True, secrets_file=a.secrets_file).decode())
    elif a.command == 'log':
        from validate_telemetry import validate
        raw = request(a.host, '/api/log', auth=True, timeout=120, secrets_file=a.secrets_file)
        clean, count = validate(raw)
        dest = a.output or ROOT / 'telemetry_logs' / time.strftime('bal_%Y%m%d_%H%M%S_wifi.csv')
        dest.parent.mkdir(parents=True, exist_ok=True)
        with dest.with_suffix('.wire').open('xb') as f:
            f.write(raw)
        with dest.open('xb') as f:
            f.write(clean)
        print(f'Validated {count} samples; saved {dest}')
    elif a.command == 'ota':
        try:
            deploy(a.host, a.firmware, a.manifest, a.record_dir, a.secrets_file, a.upload_profile)
        except (OSError, ValueError, http.client.HTTPException) as exc:
            raise SystemExit(str(exc)) from None


if __name__ == '__main__':
    main()
