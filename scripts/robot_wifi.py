#!/usr/bin/env python3
"""Hopscotch LAN telemetry, verified log download and application-only OTA."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def device_token():
    token = os.environ.get('HOPSCOTCH_API_TOKEN')
    if token:
        return token
    local = ROOT / 'src/network_secrets.h'
    if local.exists():
        match = re.search(r'^#define HOPSCOTCH_API_TOKEN "([^"\n]+)"', local.read_text(), re.M)
        if match:
            return match[1]
    raise ValueError('Set HOPSCOTCH_API_TOKEN or configure src/network_secrets.h')


def request(host, path, data=None, headers=None, auth=False, timeout=30):
    h = dict(headers or {})
    if auth:
        h['Authorization'] = 'Bearer ' + device_token()
    url = host.rstrip('/') + path
    return urllib.request.urlopen(urllib.request.Request(url, data=data, headers=h), timeout=timeout).read()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='http://hopscotch.local')
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    sub.add_parser('disarm')
    sub.add_parser('reconnect')
    log = sub.add_parser('log')
    log.add_argument('--output', type=Path)
    ota = sub.add_parser('ota')
    ota.add_argument('firmware', type=Path)
    a = p.parse_args()
    if a.command == 'status':
        print(json.dumps(json.loads(request(a.host, '/api/telemetry')), indent=2))
    elif a.command == 'disarm':
        print(request(a.host, '/api/disarm', b'', auth=True).decode())
    elif a.command == 'reconnect':
        print(request(a.host, '/api/wifi/reconnect', b'', auth=True).decode())
    elif a.command == 'log':
        from validate_telemetry import validate
        raw = request(a.host, '/api/log', auth=True, timeout=120)
        clean, count = validate(raw)
        dest = a.output or ROOT / 'telemetry_logs' / time.strftime('bal_%Y%m%d_%H%M%S_wifi.csv')
        dest.parent.mkdir(parents=True, exist_ok=True)
        with dest.with_suffix('.wire').open('xb') as f:
            f.write(raw)
        with dest.open('xb') as f:
            f.write(clean)
        print(f'Validated {count} samples; saved {dest}')
    elif a.command == 'ota':
        image = a.firmware.read_bytes()
        if len(image) < 1024 or image[0] != 0xE9:
            p.error('Use an ESP application firmware.bin, not a filesystem or merged image')
        if image[23] != 1 or hashlib.sha256(image[:-32]).digest() != image[-32:]:
            p.error('Application must include a valid ESP image SHA-256 trailer')
        state = json.loads(request(a.host, '/api/telemetry'))
        if not state.get('maintenance_allowed') or state.get('maintenance'):
            p.error('Disarm drive and arms and wait for log save before OTA')
        before_info = json.loads(request(a.host, '/api/info'))
        digest = hashlib.sha256(image).hexdigest()
        boundary = 'hopscotch-' + os.urandom(12).hex()
        body = (f'--{boundary}\r\nContent-Disposition: form-data; name="firmware"; filename="firmware.bin"\r\n'
                'Content-Type: application/octet-stream\r\n\r\n').encode() + image + f'\r\n--{boundary}--\r\n'.encode()
        print(f'Uploading {len(image)} bytes, SHA-256 {digest}', flush=True)
        response = request(a.host, '/api/ota', body, {
            'Content-Type': f'multipart/form-data; boundary={boundary}',
            'X-Firmware-Size': str(len(image)), 'X-Firmware-SHA256': digest,
        }, auth=True, timeout=120)
        print(response.decode(), flush=True)
        for _ in range(60):
            time.sleep(1)
            try:
                state = json.loads(request(a.host, '/api/telemetry', timeout=2))
                if state.get('uptime_ms', 0) > 0 and not state.get('maintenance'):
                    info = json.loads(request(a.host, '/api/info'))
                    if info['running_slot'] == before_info['running_slot']:
                        continue
                    if info.get('image_sha256') and info['image_sha256'] != image[-32:].hex():
                        raise SystemExit('Rebooted image hash does not match the uploaded application')
                    print(f"Reconnected, slot {info['running_slot']}, build {info['build']}; both groups disarmed: "
                          f"{not state['drive_armed'] and not state['arm_armed']}")
                    return
            except (OSError, ValueError):
                pass
        raise SystemExit('Upload accepted but reboot not verified. Check the robot before use.')


if __name__ == '__main__':
    main()
