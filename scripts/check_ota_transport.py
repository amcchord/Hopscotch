#!/usr/bin/env python3
"""Exercise actual OTA methods and the pinned TCP poll implementation on the host.

Run after dependency bootstrap/build. No robot access or private configuration.
"""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
        end += 1
    return source[start:end]


def main():
    source = (ROOT / 'src/web_server.cpp').read_text()
    tcp = ROOT / '.pio/libdeps/m5stack-atoms3r/Async TCP/src/AsyncTCP.cpp'
    http = ROOT / '.pio/libdeps/m5stack-atoms3r/ESP Async WebServer/src/WebServer.cpp'
    if not tcp.exists() or not http.exists():
        raise SystemExit('Bootstrap pinned dependencies first: pio pkg install --no-save')
    assert 'c->setRxTimeout(3);' in http.read_text(), 'Review changed HTTP default timeout'
    constants = '\n'.join(re.findall(r'^constexpr uint32_t OTA_.*;$', source, re.M))
    watchdog = re.search(r'if \(_ota_request && !_restart_ms && millis\(\) - _ota_last_ms > OTA_IDLE_TIMEOUT_MS\)\s*failOta\("inactivity_timeout"\);', source)
    assert watchdog, 'Review changed OTA watchdog integration'
    implementations = '\n'.join([
        constants, function(tcp.read_text(), 'int8_t AsyncClient::_poll('),
        function(source, 'void WebUI::failOta('), function(source, 'void WebUI::upload('),
        'void WebUI::serviceWatchdog() { ' + watchdog[0] + ' }',
    ])
    harness = (ROOT / 'tests/ota_transport_harness.cpp').read_text()
    output = ROOT / 'output/ota-transport-test'
    output.mkdir(parents=True, exist_ok=True)
    unit = output / 'test.cpp'
    unit.write_text(harness.replace('// IMPLEMENTATIONS', implementations))
    exe = output / 'test'
    subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(ROOT / 'src'), str(unit), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
