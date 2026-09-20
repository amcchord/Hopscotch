"""Run production OTA callbacks and the control interlock with hardware stubs."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class OtaLifecycleTests(unittest.TestCase):
    def test_upload_and_control_interlock(self):
        source = (ROOT / "src/web_server.cpp").read_text()
        control = function((ROOT / "src/main.cpp").read_text(), "static void controlTick() {")
        interlock = control[control.index("    // All safety predicates"):control.index("    // Process serial debug commands")]
        watchdog = re.search(r"if \(_ota_request && !_restart_ms && millis\(\) - _ota_last_ms > [^\n]+\)\s*failOta\([^;]*;", source)
        self.assertIsNotNone(watchdog)
        constants = re.findall(r"constexpr uint32_t OTA_[A-Z_]+ = [0-9]+;", source)
        self.assertEqual(len(constants), 3)
        implementations = "\n".join(constants + [
            function(source, "OtaProgress WebUI::otaProgress("),
            function(source, "void WebUI::publishOta("),
            function(source, "void WebUI::failOta("),
            function(source, "void WebUI::upload("),
            "void WebUI::watchdog() { " + watchdog[0] + " }",
            "void controlOnce() { uint32_t now = millis();\n" + interlock + "\n++motionTicks;\n}",
        ])
        code = (ROOT / "tests/ota_lifecycle_harness.cpp").read_text().replace("// IMPLEMENTATIONS", implementations)
        output = ROOT / "output"
        output.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=output, prefix="ota-lifecycle-") as folder:
            unit = Path(folder) / "test.cpp"
            unit.write_text(code)
            exe = Path(folder) / "test"
            subprocess.run(["clang++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I" + str(ROOT / "tests/stubs"), "-I" + str(ROOT / "src"),
                            str(unit), str(ROOT / "src/crsf.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
