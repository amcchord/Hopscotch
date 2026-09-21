"""Host-side deployment regressions; no robot or firmware build required."""
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import robot_wifi as wifi


def image_bytes():
    content = bytearray(1024)
    content[0], content[23] = 0xE9, 1
    return bytes(content) + hashlib.sha256(content).digest()


def idle():
    return dict(maintenance_allowed=True, maintenance=False, drive_armed=False, arm_armed=False,
                arming=False, saving_log=False, calibration=False, test_mode=False, simulation=False,
                age_ms=5, balance=dict(state='IDLE', active=False, fault=0, imu_age_us=1000),
                motors=[dict(id=i, enabled=False, online=True, error=0, age_ms=30) for i in range(6)])


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.firmware = self.directory / 'firmware.bin'
        self.firmware.write_bytes(image_bytes())
        self.identity = wifi.application_identity(image_bytes())
        self.old = dict(running_slot='app1', image_sha256='old', ota_capacity=3342336, build='old')
        self.new = dict(self.old, running_slot='app0', image_sha256=self.identity['esp_image_digest'], build='new')

    def test_manifest_and_image_integrity_checked_before_device_access(self):
        for key in ('application_bytes', 'application_sha256', 'esp_image_digest'):
            manifest = dict(self.identity)
            manifest[key] = 'wrong'
            path = self.directory / 'manifest.json'
            path.write_text(json.dumps(manifest))
            with patch.object(wifi, 'request') as request, self.assertRaisesRegex(ValueError, 'manifest mismatch'):
                wifi.deploy('http://robot', self.firmware, path)
            request.assert_not_called()
        damaged = bytearray(image_bytes())
        damaged[100] ^= 1
        with self.assertRaisesRegex(ValueError, 'SHA-256 trailer'):
            wifi.application_identity(damaged)

    def test_all_unsafe_states_block_client_preflight(self):
        for field in ('maintenance', 'drive_armed', 'arm_armed', 'arming', 'saving_log', 'calibration', 'test_mode', 'simulation'):
            state = idle()
            state[field] = True
            with self.subTest(field=field), self.assertRaises(ValueError):
                wifi.require_idle(state)
        for alter in (lambda s: s.pop('maintenance_allowed'),
                      lambda s: s.update(age_ms=1000),
                      lambda s: s['balance'].update(imu_age_us=100000),
                      lambda s: s['balance'].update(fault=1),
                      lambda s: s['balance'].update(active=True),
                      lambda s: s['motors'][0].update(enabled=True),
                      lambda s: s['motors'].pop()):
            state = idle()
            alter(state)
            with self.assertRaises(ValueError):
                wifi.require_idle(state)

    def test_powered_feedback_must_return_but_offline_update_is_allowed(self):
        state = idle()
        for m in state['motors']:
            m['online'] = False
        wifi.require_idle(state)
        with self.assertRaisesRegex(ValueError, 'Powered motor'):
            wifi.require_idle(state, [0])

    def test_hash_missing_wrong_slot_and_armed_postflight_never_succeed(self):
        cases = [(dict(self.new, image_sha256=None), idle()),
                 (dict(self.new, running_slot='app1'), idle()),
                 (self.new, dict(idle(), arm_armed=True))]
        for info, state in cases:
            with self.subTest(info=info, armed=state['arm_armed']), \
                    patch.object(wifi, 'request', side_effect=[json.dumps(info), json.dumps(state)]), \
                    patch.object(wifi.time, 'monotonic', side_effect=[0, 0, 61]), \
                    patch.object(wifi.time, 'sleep'), self.assertRaisesRegex(ValueError, 'not verified'):
                wifi.wait_for_image('http://robot', self.identity['esp_image_digest'], 'app1')

    def run_deploy(self, before=None, logs=None, transfer=None, profile='paced'):
        before = before or self.old
        logs = logs or [dict(samples=5, csv_sha256='csv', wire_sha256='wire')] * 2
        def request(_host, path, **kwargs):
            return json.dumps(before if path == '/api/info' else idle())
        with patch.object(wifi, 'device_token', return_value='private-test-token'), \
                patch.object(wifi, 'request', side_effect=request), \
                patch.object(wifi, 'archive_log', side_effect=logs) as archive, \
                patch.object(wifi, 'upload_application', return_value=transfer or dict(status=200)) as upload, \
                patch.object(wifi, 'wait_for_image', return_value=(self.new, idle())):
            result = wifi.deploy('http://robot', self.firmware, record_dir=self.directory / 'record', upload_profile=profile)
            if upload.called:
                self.assertEqual(upload.call_args.kwargs, wifi.UPLOAD_PROFILES[profile])
            return result, archive.call_count, upload.call_count

    def test_fast_profile_keeps_preflight_backup_and_postflight(self):
        record, archives, uploads = self.run_deploy(profile='fast')
        self.assertEqual(record['upload_profile'], 'fast')
        self.assertEqual(record['status'], 'installed_verified')
        self.assertEqual((archives, uploads), (2, 1))

    def test_invalid_profile_and_pacing_fail_before_network_access(self):
        with patch.object(wifi, 'request') as request, self.assertRaises(ValueError):
            wifi.deploy('http://robot', self.firmware, upload_profile='bogus')
        request.assert_not_called()
        for kwargs in (dict(interval=-1), dict(interval=float('nan')), dict(interval=6),
                       dict(chunk_bytes=0), dict(chunk_bytes=65537)):
            with self.subTest(kwargs=kwargs), patch.object(wifi.http.client, 'HTTPConnection') as connection, self.assertRaises(ValueError):
                wifi.upload_application('http://robot', image_bytes(), self.identity, **kwargs)
            connection.assert_not_called()

    def test_fast_and_paced_uploads_measure_backpressure_and_sleep(self):
        payload = image_bytes() * 40
        for profile in ('fast', 'paced'):
            with self.subTest(profile=profile), \
                    patch.object(wifi, 'device_token', return_value='private-test-token'), \
                    patch.object(wifi.http.client, 'HTTPConnection') as connection:
                conn = connection.return_value
                now = [0.0]
                sent = []
                def send(chunk):
                    sent.append(chunk)
                    now[0] += .02  # Simulated socket backpressure.
                def sleep(interval):
                    now[0] += interval
                conn.send.side_effect = send
                response = conn.getresponse.return_value
                response.status = 200
                response.read.return_value = b'Firmware verified; rebooting'
                headers = {'X-OTA-Write-Us': '12345', 'X-OTA-Verify-Us': 'bad', 'X-OTA-Elapsed-Ms': '120'}
                response.getheader.side_effect = headers.get
                with patch.object(wifi.time, 'monotonic', side_effect=lambda: now[0]), \
                        patch.object(wifi.time, 'sleep', side_effect=sleep) as sleeper:
                    result = wifi.upload_application('http://robot', payload, self.identity, **wifi.UPLOAD_PROFILES[profile])
                body = b''.join(sent)
                self.assertEqual(result['status'], 200)
                self.assertIn(payload, body)
                self.assertEqual(result['sent_bytes'], len(body))
                self.assertAlmostEqual(result['send_block_seconds'], len(sent) * .02)
                self.assertEqual(result['max_send_block_seconds'], .02)
                self.assertEqual(result['server_timings'], {'write_us': 12345, 'elapsed_ms': 120})
                if profile == 'fast':
                    sleeper.assert_not_called()
                    self.assertEqual(len(sent[0]), 16384)
                    self.assertEqual(result['pacing_sleep_seconds'], 0)
                else:
                    self.assertEqual(len(sent[0]), 1024)
                    self.assertAlmostEqual(result['pacing_sleep_seconds'], .05 * (len(sent) - 1))

    def test_fast_disconnect_never_retries_or_switches_profile(self):
        with patch.object(wifi, 'device_token', return_value='private-test-token'), \
                patch.object(wifi.http.client, 'HTTPConnection') as connection:
            conn = connection.return_value
            conn.send.side_effect = OSError('connection lost')
            conn.getresponse.side_effect = OSError('no response')
            result = wifi.upload_application('http://robot', image_bytes(), self.identity, **wifi.UPLOAD_PROFILES['fast'])
            self.assertIn('connection lost', result['transport_error'])
            self.assertEqual(result['sent_bytes'], 0)
            self.assertEqual(conn.send.call_count, 1)
            self.assertEqual(connection.call_count, 1)

    def test_response_timeout_closes_once_without_reusing_failed_reader(self):
        with patch.object(wifi, 'device_token', return_value='private-test-token'), \
                patch.object(wifi.http.client, 'HTTPConnection') as connection:
            conn = connection.return_value
            conn.getresponse.side_effect = TimeoutError('timed out')
            result = wifi.upload_application('http://robot', image_bytes(), self.identity,
                                             **wifi.UPLOAD_PROFILES['fast'])
            self.assertEqual(result['transport_error'], 'timed out')
            self.assertGreater(result['sent_bytes'], len(image_bytes()))
            conn.getresponse.assert_called_once()
            conn.close.assert_called_once()
            self.assertEqual(connection.call_count, 1)

    def test_early_http_rejection_is_retained_after_body_send_failure(self):
        with patch.object(wifi, 'device_token', return_value='private-test-token'), \
                patch.object(wifi.http.client, 'HTTPConnection') as connection:
            conn = connection.return_value
            conn.send.side_effect = BrokenPipeError('body rejected')
            response = conn.getresponse.return_value
            response.status = 409
            response.read.return_value = b'Maintenance denied'
            response.getheader.return_value = None
            result = wifi.upload_application('http://robot', image_bytes(), self.identity)
            self.assertEqual(result['status'], 409)
            self.assertEqual(result['response'], 'Maintenance denied')
            conn.getresponse.assert_called_once()
            self.assertEqual(result['sent_bytes'], 0)

    def test_lost_upload_response_verifies_without_retransmitting(self):
        record, archives, uploads = self.run_deploy(transfer=dict(transport_error='connection reset'))
        self.assertEqual(record['status'], 'installed_verified')
        self.assertEqual((archives, uploads), (2, 1))
        self.assertNotIn('private-test-token', (self.directory / 'record/deployment.json').read_text())

    def test_already_installed_is_verified_without_reboot_or_upload(self):
        record, archives, uploads = self.run_deploy(before=self.new)
        self.assertEqual(record['status'], 'already_installed_verified')
        self.assertEqual((archives, uploads), (1, 0))

    def test_changed_saved_run_fails_handoff_and_retains_evidence(self):
        with self.assertRaisesRegex(ValueError, 'Saved run changed'):
            self.run_deploy(logs=[dict(csv_sha256='before'), dict(csv_sha256='after')])
        record = json.loads((self.directory / 'record/deployment.json').read_text())
        self.assertEqual(record['status'], 'verification_failed')
        self.assertEqual(record['saved_run_after']['csv_sha256'], 'after')

    def test_corrupt_export_is_saved_but_not_accepted(self):
        with patch.object(wifi, 'request', return_value=b'truncated data'), self.assertRaises(ValueError):
            wifi.archive_log('http://robot', self.directory, 'before')
        self.assertEqual((self.directory / 'before.wire').read_bytes(), b'truncated data')
        self.assertFalse((self.directory / 'before.csv').exists())

    def test_no_saved_log_is_explicitly_recorded(self):
        with patch.object(wifi, 'request', return_value=b'[Balance] No log file found\r\n'):
            record = wifi.archive_log('http://robot', self.directory, 'before')
        self.assertTrue(record['absent'])
        self.assertEqual(record['samples'], 0)

    def test_transport_sends_exact_multipart_and_waits_for_slow_receiver(self):
        captured = {}
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                captured.update(path=self.path, headers=dict(self.headers),
                                body=self.rfile.read(int(self.headers['Content-Length'])))
                # The image is queued locally, but the receiver still needs
                # longer than the connection/send timeout to finish it.
                time.sleep(0.15)
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'Firmware verified; rebooting')
            def log_message(self, *args):
                pass
        server = HTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.handle_request, daemon=True)
        thread.start()
        try:
            with patch.object(wifi, 'device_token', return_value='private-test-token'), \
                    patch.object(wifi, 'UPLOAD_SOCKET_TIMEOUT_SECONDS', 0.03), \
                    patch.object(wifi, 'UPLOAD_RESPONSE_TIMEOUT_SECONDS', 2):
                result = wifi.upload_application(f'http://127.0.0.1:{server.server_port}',
                                                 image_bytes(), self.identity, interval=0)
            thread.join(timeout=3)
        finally:
            server.server_close()
        self.assertEqual(result['status'], 200)
        headers, body = captured['headers'], captured['body']
        self.assertEqual(captured['path'], '/api/ota')
        self.assertEqual(headers['Authorization'], 'Bearer private-test-token')
        self.assertEqual(headers['X-Firmware-SHA256'], hashlib.sha256(image_bytes()).hexdigest())
        self.assertEqual(headers['X-Firmware-Size'], str(len(image_bytes())))
        self.assertEqual(int(headers['Content-Length']), len(body))
        boundary = headers['Content-Type'].split('boundary=')[1].encode()
        self.assertEqual(body.split(b'\r\n\r\n', 1)[1], image_bytes() + b'\r\n--' + boundary + b'--\r\n')


if __name__ == '__main__':
    unittest.main()
