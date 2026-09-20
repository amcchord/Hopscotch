"""Frozen installed v4 versus candidate source; no device I/O."""
from model import *
import itertools,json,hashlib,csv,bisect
BASE='aa9ae13'
frozen=TMP/'baseline'
for name in ('src/config.h','src/balance_pilot.h','src/balance_math.h','src/balance_drive.h','scripts/balance_sim.py','evidence/balance-drive-damping/pilot_bridge.cpp'):
 p=frozen/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(subprocess.check_output(['git','show',f'{BASE}:{name}'],cwd=ROOT))
subprocess.run(['clang++','-std=c++17','-shared','-fPIC','-O2',f'-I{frozen}/src','-Itests/stubs',str(frozen/'evidence/balance-drive-damping/pilot_bridge.cpp'),'-o',str(TMP/'v4.dylib')],cwd=ROOT,check=True)
oldlib=ct.CDLL(str(TMP/'v4.dylib'))
for name in ('pilot_create','pilot_destroy','pilot_tick','pilot_params','pilot_learning','pilot_arm_error','pilot_drive','pilot_correction','pilot_arm_demand','pilot_emergency'):
 a=getattr(lib,name);b=getattr(oldlib,name);b.argtypes=a.argtypes;b.restype=a.restype
