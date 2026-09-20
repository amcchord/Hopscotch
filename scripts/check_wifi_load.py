#!/usr/bin/env python3
"""Motors-OFF bench load: stalled WS, three readers and three HTTP workers."""
import argparse
import base64
import json
import os
from pathlib import Path
import socket
import subprocess
from contextlib import nullcontext
from urllib.parse import urlsplit
from robot_wifi import request


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='http://hopscotch.local')
    p.add_argument('--seconds', type=int, default=120)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mode', choices=['normal', 'overload'], default='overload')
    p.add_argument('--ignore-radio', action='store_true', help='Do not require an RC link during motors-off network testing')
    a = p.parse_args()
    if a.seconds < 30:
        p.error('Use at least 30 seconds to exercise TCP backpressure')
    if a.output.exists():
        p.error('Output already exists; retain earlier evidence')
    state = json.loads(request(a.host, '/api/telemetry'))
    if not state['maintenance_allowed'] or state['maintenance'] or any(m['online'] for m in state['motors']):
        p.error('This bench test requires disarmed robot and motor power OFF')
    u = urlsplit(a.host)
    if u.scheme != 'http':
        p.error('Use the robot HTTP LAN address')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket() if a.mode == 'overload' else nullcontext() as sock:
        if sock is not None:
            stalled_client(sock, u)
        subprocess.run(['node', str(Path(__file__).with_suffix('.mjs')),
                        a.host.rstrip('/'), str(a.seconds), str(a.output), a.mode,
                        'ignore-radio' if a.ignore_radio else 'require-radio'], check=True)
    x = json.loads(a.output.read_text())
    assert not x['errors'], x['errors']
    assert min(x['websocket_frames']) > a.seconds, x['websocket_frames']
    assert x['after']['uptime_ms'] - x['before']['uptime_ms'] >= a.seconds * 990, 'Device reset'
    for name in ['balance_200hz', 'control_200hz']:
        before, after = x['before']['timing'][name], x['after']['timing'][name]
        assert after['over_7500_us'] == before['over_7500_us'], (name, before, after)
        assert after['ticks'] - before['ticks'] >= a.seconds * 195, (name, before, after)
    if a.mode == 'overload':
        assert x['after']['ws_dropped'] > x['before']['ws_dropped'], 'Backpressure was not exercised'
        # Saturation may evict TCP connections. Demand automatic recovery and
        # successful delivery throughout, while control remains uninterrupted.
        assert max(x['max_frame_gap_ms']) < 15000, x['max_frame_gap_ms']
        assert len(x['transport_errors']) < max(1, x['http_requests'] * .05), x['transport_errors']
    else:
        assert not x['transport_errors'] and not x['reconnects'], x['transport_errors']
        assert max(x['max_frame_gap_ms']) < 2000, x['max_frame_gap_ms']
        assert min(x['websocket_frames']) > a.seconds * 4, x['websocket_frames']
    print(f'PASS: {a.output}')


def stalled_client(sock, u):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
    sock.settimeout(8)
    sock.connect((u.hostname, u.port or 80))
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((f'GET /ws HTTP/1.1\r\nHost: {u.netloc}\r\nUpgrade: websocket\r\n'
                  f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
                  'Sec-WebSocket-Version: 13\r\n\r\n').encode())
    header = b''
    while not header.endswith(b'\r\n\r\n'):
        b = sock.recv(1)
        if not b or len(header) > 8192:
            raise RuntimeError('WebSocket upgrade failed')
        header += b
    if b'101 Switching Protocols' not in header:
        raise RuntimeError('WebSocket upgrade rejected')
    # Intentionally never consume data: the TCP receive window must fill.

if __name__ == '__main__':
    main()
