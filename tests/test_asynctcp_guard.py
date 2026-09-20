"""Exercise the actual dependency callback replacement with a stub listener."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('patch', ROOT / 'scripts/patch_asynctcp.py')
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)


class AcceptGuardTests(unittest.TestCase):
    def test_rejects_changed_dependency(self):
        with self.assertRaises(RuntimeError):
            patch.patched_source(patch.OLD)

    def test_invalid_callbacks_do_not_dereference_listener(self):
        code = '''#include <cstdint>
#include <cassert>
#include <cstddef>
constexpr int8_t ERR_OK=0, ERR_VAL=-6, ERR_MEM=-1;
struct tcp_pcb {};
struct AsyncServer {
    int calls=0;
    int8_t _accept(tcp_pcb*, int8_t) { ++calls; return 42; }
    static int8_t _s_accept(void*, tcp_pcb*, int8_t);
};
struct AsyncClient {
    tcp_pcb* _pcb = reinterpret_cast<tcp_pcb*>(1); // Freed memory: must never read.
    int freed=0;
    void _free_closed_slot() { ++freed; }
    void (*_error_cb)(void*, AsyncClient*, int8_t) = nullptr;
    void (*_discard_cb)(void*, AsyncClient*) = nullptr;
    void* _error_cb_arg=nullptr; void* _discard_cb_arg=nullptr;
    void _error(int8_t);
};
''' + patch.NEW + patch.ERROR_NEW + '''
int main() {
    AsyncServer s; tcp_pcb pcb;
    assert(AsyncServer::_s_accept(nullptr, &pcb, ERR_OK)==ERR_VAL);
    assert(AsyncServer::_s_accept(&s, nullptr, ERR_MEM)==ERR_VAL);
    assert(AsyncServer::_s_accept(&s, &pcb, ERR_MEM)==ERR_MEM);
    assert(s.calls==0);
    assert(AsyncServer::_s_accept(&s, &pcb, ERR_OK)==42);
    assert(s.calls==1);
    AsyncClient c; int errors=0, discards=0;
    c._error_cb_arg=&errors; c._discard_cb_arg=&discards;
    c._error_cb=[](void* p, AsyncClient* c, int8_t err) {
        assert(c->_pcb==nullptr && c->freed==1 && err==ERR_MEM);
        ++*static_cast<int*>(p);
    };
    c._discard_cb=[](void* p, AsyncClient* c) {
        assert(c->_pcb==nullptr); ++*static_cast<int*>(p);
    };
    c._error(ERR_MEM);
    assert(errors==1 && discards==1);
}
'''
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / 'guard.cpp'
            source.write_text(code)
            exe = Path(d) / 'guard'
            subprocess.run(['clang++', '-std=c++17', '-Wall', '-Werror', str(source), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
