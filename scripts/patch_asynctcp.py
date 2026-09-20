"""Reproducible, fail-closed fixes for Async TCP 3.1.4's error callbacks.

The motors-off load test crashed in _s_accept -> _accept with arg == nullptr.
Reject invalid callbacks before dereferencing the listener. lwIP's SYN_RCVD
caller aborts the PCB when the callback returns an error other than ERR_ABRT;
the callback must not also free it. Also remove access to the freed PCB from
_error: lwIP explicitly frees it before invoking tcp_err, and another handshake
can reuse its memory before the async task processes the error notification.
Do not silently apply to another version.
"""
import hashlib
from pathlib import Path

ORIGINAL_SHA256 = '0a436501abfb6aedc68c1457682b7a53986a2ab3123365d6ad09823e5f7f6723'
OLD = '''int8_t AsyncServer::_s_accept(void * arg, tcp_pcb * pcb, int8_t err){
    return reinterpret_cast<AsyncServer*>(arg)->_accept(pcb, err);
}'''
NEW = '''int8_t AsyncServer::_s_accept(void * arg, tcp_pcb * pcb, int8_t err){
    // Hopscotch: a stale/failed handshake must not reboot motor control.
    // lwIP owns PCB disposal after a non-ERR_ABRT callback failure.
    if (!arg || !pcb) return ERR_VAL;
    if (err != ERR_OK) return err;
    return reinterpret_cast<AsyncServer*>(arg)->_accept(pcb, err);
}'''
ERROR_OLD = '''void AsyncClient::_error(int8_t err) {
    if(_pcb){
        tcp_arg(_pcb, NULL);
        if(_pcb->state == LISTEN) {
            tcp_sent(_pcb, NULL);
            tcp_recv(_pcb, NULL);
            tcp_err(_pcb, NULL);
            tcp_poll(_pcb, NULL, 0);
        }
        _free_closed_slot();
        _pcb = NULL;
    }
    if(_error_cb) {
        _error_cb(_error_cb_arg, this, err);
    }
    if(_discard_cb) {
        _discard_cb(_discard_cb_arg, this);
    }
}'''
ERROR_NEW = '''void AsyncClient::_error(int8_t err) {
    // Hopscotch: lwIP already freed this PCB before its error callback.
    // It may now belong to another connection. Never dereference it here.
    _pcb = NULL;
    _free_closed_slot();
    if(_error_cb) {
        _error_cb(_error_cb_arg, this, err);
    }
    if(_discard_cb) {
        _discard_cb(_discard_cb_arg, this);
    }
}'''


def patched_source(source):
    original = source.replace(NEW, OLD).replace(ERROR_NEW, ERROR_OLD)
    if hashlib.sha256(original.encode()).hexdigest() != ORIGINAL_SHA256:
        raise RuntimeError('Async TCP source changed; review the accept guard before building')
    if original.count(OLD) != 1 or original.count(ERROR_OLD) != 1:
        raise RuntimeError('Expected exactly one of each Async TCP callback')
    return original.replace(OLD, NEW).replace(ERROR_OLD, ERROR_NEW)


if 'Import' in globals():
    Import('env')  # type: ignore[name-defined]  # PlatformIO/SCons post script
    path = Path(env.subst('$PROJECT_LIBDEPS_DIR/$PIOENV/Async TCP/src/AsyncTCP.cpp'))
    source = path.read_text()
    patched = patched_source(source)
    if patched != source:
        path.write_text(patched)
        print('Applied pinned Async TCP accept and freed-PCB guards')
