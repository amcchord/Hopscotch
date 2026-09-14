import io
import sys
import unittest
from collections import deque
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from receive_telemetry import receive


class FakePort:
    def __init__(self, chunks): self.chunks = deque(chunks)
    @property
    def in_waiting(self): return len(self.chunks[0]) if self.chunks and isinstance(self.chunks[0], bytes) else 1
    def read(self, size):
        if not self.chunks: return b''
        chunk = self.chunks.popleft()
        if isinstance(chunk, Exception): raise chunk
        return chunk


class ReceiverTests(unittest.TestCase):
    def test_fragmented_trailer_preserves_exact_bytes(self):
        chunks = [b'# === BALANCE CONFIG ===\r\n', b't_ms,state\n20,2\r\n[Balance] --- E', b'nd of log ---\r', b'\n']
        output = io.BytesIO()
        self.assertEqual(receive(FakePort(chunks), output, 5), len(b''.join(chunks)))
        self.assertEqual(output.getvalue(), b''.join(chunks))

    def test_disconnect_preserves_partial_data(self):
        output = io.BytesIO()
        with self.assertRaises(OSError):
            receive(FakePort([b't_ms,state\n20,2\n', OSError('disconnected')]), output, 5)
        self.assertEqual(output.getvalue(), b't_ms,state\n20,2\n')

    def test_aborted_device_dump_does_not_wait_for_timeout(self):
        output = io.BytesIO()
        with self.assertRaisesRegex(ValueError, 'resumed idle'):
            receive(FakePort([b't_ms,sta', b'te\n[Loop] t=100\n']), output, 5)

    def test_timeout_and_refusal(self):
        ticks = iter([0, 0, 2])
        with self.assertRaises(TimeoutError): receive(FakePort([]), io.BytesIO(), 1, lambda: next(ticks))
        with self.assertRaisesRegex(ValueError, 'REFUSED'):
            receive(FakePort([b'[Balance] Log dump REFUSED\n']), io.BytesIO(), 5)
