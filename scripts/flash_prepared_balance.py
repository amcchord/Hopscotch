#!/usr/bin/env python3
"""Verify a frozen package; flash only when explicitly invoked with --flash.

Programs only app0 at 0x10000 using the project's existing ESP32-S3 USB JTAG
transport. Does not erase/update partitions, bootloader, NVS or LittleFS.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, default=ROOT / 'artifacts/balance-candidate')
    parser.add_argument('--rollback', action='store_true', help='select the rebuilt e8b1280 application')
    parser.add_argument('--flash', action='store_true', help='actually program the attached, disarmed robot')
    args = parser.parse_args()
    package = args.package.resolve()
    manifest = json.loads((package / 'manifest.json').read_text())
    # Validate every supplied file, not just the selected image.
    for name, expected in manifest['files'].items():
        path = (package / name).resolve()
        if not path.is_relative_to(package):
            raise SystemExit('Package contains a path outside its directory')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected['sha256']:
            raise SystemExit(f'Checksum mismatch: {name}')
    name = 'rollback/firmware.bin' if args.rollback else 'firmware.bin'
    if name not in manifest['files']:
        raise SystemExit(f'Package does not identify {name}')
    image = package / name
    if manifest['application_offset'] != '0x10000' or manifest['target'] != 'esp32s3':
        raise SystemExit('Package target or application offset does not match this flasher')
    if any(c in str(image) for c in '{}\n\r'):
        raise SystemExit('Unsupported characters in package path')
    # Match the tool pinned by the validated PlatformIO 6.7.0 environment.
    packages = Path.home() / '.platformio/packages'
    matches = []
    for directory in packages.glob('tool-openocd-esp32*'):
        meta = directory / 'package.json'
        if meta.is_file() and json.loads(meta.read_text()).get('version') == '2.1100.20220706':
            matches.append(directory)
    if not matches:
        raise SystemExit('Validated OpenOCD package missing; run scripts/build.sh to install build dependencies')
    tool = matches[0]
    executable = tool / 'bin/openocd'
    scripts = tool / 'share/openocd/scripts'
    for path in (executable, scripts/'interface/esp_usb_jtag.cfg', scripts/'target/esp32s3.cfg'):
        if not path.is_file():
            raise SystemExit(f'Missing upload dependency: {path}')
    command = [str(executable), '-s', str(scripts), '-f', 'interface/esp_usb_jtag.cfg',
               '-f', 'target/esp32s3.cfg', '-c', 'adapter speed 5000',
               '-c', f'program_esp {{{image}}} 0x10000 verify', '-c', 'reset run; shutdown']
    print(f"Verified package source: {manifest['source_commit']}")
    print(f'Selected image: {image}')
    print(f"SHA-256: {manifest['files'][name]['sha256']}")
    if not args.flash:
        print('Verification only; no USB device accessed. Add --flash after device backup and disarm.')
        return
    print('Programming and verifying application only; robot must already be supported and disarmed.', flush=True)
    subprocess.run(command, check=True)


if __name__ == '__main__':
    main()
