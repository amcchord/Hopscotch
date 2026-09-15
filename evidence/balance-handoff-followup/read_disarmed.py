"""Read-only status capture; never sends arm or motion commands."""
import argparse,time,re
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args()
with serial.Serial('/dev/cu.usbmodem2101',115200,timeout=.1,write_timeout=1) as s:
    start=time.monotonic();sent=set();data=bytearray()
    with Path(a.output).open('wb') as f:
        while time.monotonic()-start<8:
            t=time.monotonic()-start
            for at,command in ((1,b'bal status\n'),(3,b'status\n'),(5,b'cal status\n')):
                if t>=at and at not in sent:
                    s.write(command);sent.add(at)
            chunk=s.read(max(1,min(s.in_waiting,8192)))
            if chunk:f.write(chunk);f.flush();data.extend(chunk)
text=data.decode(errors='replace')
states=re.findall(r'drv_armed=(\d) arm_armed=(\d)',text)
assert states and all(s==('0','0') for s in states), 'Disarmed preflight failed'
for line in text.splitlines():
    if any(x in line for x in ('State:','Stored trim:','IMU age:','Complementary filter:','[CRSF] RX','[CAN]','[Status] link','calibrat','center:','forward:','back:')):print(line)
print('Verified both groups disarmed in',len(states),'status lines')
