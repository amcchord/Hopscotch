"""Passive observation with a descriptive note; never arms or requests motion."""
import time
from pathlib import Path
import serial

out = Path(__file__).with_name('first-trial.serial')
with serial.Serial('/dev/cu.usbmodem2101',115200,timeout=.1,write_timeout=1) as s, out.open('xb') as f:
    s.write(b'bal note confirmed-recoil-release-v1\n')
    start=time.monotonic();tail=bytearray();saw_run=False
    print('Recorder open; waiting for the operator.',flush=True)
    while time.monotonic()-start<240:
        b=s.read(max(1,min(s.in_waiting,8192)))
        if not b:continue
        f.write(b);f.flush();tail.extend(b);tail=tail[-32768:]
        if b'[Balance] TIPPING UP' in tail or b'[Balance] BALANCING' in tail:saw_run=True
        if saw_run and b'[Balance] Log saved and checksummed' in tail:
            print('Run ended; onboard log saved.',flush=True);break
    print('Recorder closed.',flush=True)
