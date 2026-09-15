import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from validate_telemetry import validate, fnv1a
from balance_sim import current_firmware_config


def transfer(rows=b'20,2,0.010,1\r\n40,2,0.020,2\r\n', count=2):
    body = (b'# === BALANCE CONFIG ===\r\n# transport_checksum=fnv1a32\n'
            b'# telemetry_features=1\n# telemetry_schema=2\n# checksum_valid=1\n'
            + f'# sample_count={count}\n'.encode()
            + b't_ms,state,roll,imu_age_ms\r\n' + rows)
    return body + f'# transport_fnv1a=0x{fnv1a(body):08X}\n'.encode() + b'[Balance] --- End of log ---\r\n'


class TelemetryTests(unittest.TestCase):
    def test_valid_mixed_newlines(self):
        clean, n = validate(transfer())
        self.assertEqual(n, 2)
        self.assertTrue(clean.endswith(b'40,2,0.020,2\n'))
    def test_silent_numeric_corruption(self):
        with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
            validate(transfer().replace(b'0.010', b'9.010'))
    def test_missing_trailer(self):
        with self.assertRaises(ValueError):
            validate(transfer().split(b'# transport_fnv1a')[0])
    def test_truncated_rows_even_with_valid_checksum(self):
        with self.assertRaisesRegex(ValueError, 'Truncated'):
            validate(transfer(rows=b'20,2,0.010,1\r\n'))
    def test_malformed_row(self):
        with self.assertRaisesRegex(ValueError, 'fields'):
            validate(transfer(rows=b'20,2,0.010\r\n', count=1))
    def test_nonfinite_rejected(self):
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            validate(transfer(rows=b'20,2,nan,1\r\n', count=1))
    def test_backward_timestamp(self):
        with self.assertRaisesRegex(ValueError, 'backward'):
            validate(transfer(rows=b'40,2,1,1\r\n20,2,1,1\r\n'))
    def test_interleaved_debug_detected(self):
        with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
            validate(transfer().replace(b'40,2,', b'[debug] noisy\n40,2,'))
    def test_pre_extension_v2_unknown_age(self):
        raw = b'# telemetry_schema=2\n# checksum_valid=1\n# sample_count=1\nt_ms,state,imu_age_ms\n20,2,\n[Balance] --- End of log ---\n'
        self.assertEqual(validate(raw)[1], 1)
    def test_legacy(self):
        raw = b't_ms,state,roll\n20,2,83.1\n[Balance] --- End of log ---\n'
        self.assertEqual(validate(raw)[1], 1)
    def test_full_120_second_capture(self):
        rows = b''.join(f'{i*20},2,84.5,1\r\n'.encode() for i in range(6000))
        self.assertEqual(validate(transfer(rows, 6000))[1], 6000)
    def test_simulator_reads_candidate_settings(self):
        c = current_firmware_config()
        self.assertEqual(c.ramp_off_clamp, 1.5)
        self.assertAlmostEqual(c.drift_vel_kp, 0.05*50/33)
        self.assertEqual(c.arm_emergency_cmd_frac, 0.45)
        self.assertEqual(c.arm_calm_ms, 300)
        self.assertEqual(c.base_sp_rate_max, 4)
        self.assertEqual(c.arm_return_acceleration, 0)
        self.assertEqual(c.arm_return_speed, 1.5)
        self.assertTrue(c.absolute_capture_trim)
        self.assertAlmostEqual(c.vel_sp_kp, 2.2*33/50)
        self.assertTrue(c.measured_arm_arrival and c.hard_stop_latches)

if __name__ == '__main__':
    unittest.main()
