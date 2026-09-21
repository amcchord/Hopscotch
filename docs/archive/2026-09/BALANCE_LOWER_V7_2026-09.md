# CH11 lowering v7 — leave driving mode and continue through rocking contact

> Historical record, archived 2026-09-21. Use [CURRENT](../../progress/CURRENT.md)
> and [the OTA guide](../../WIFI_OTA.md) for present identity, ownership and commands.

**Physical result:** [V7 succeeded](../../../evidence/balance-lower/trial-v7-success-20260920/README.md), finishing flat with arms Forward in 14.109 s after CH11. [V8](BALANCE_LOWER_V8_2026-09.md) prepares a faster supported return while preserving the successful catch and landing.

Successful physical source `c442e12`; [current state](../../progress/CURRENT.md)
identifies the installed image. The [v6 physical run](../../../evidence/balance-lower/trial-v6-20260920/README.md)
confirms successful fast standing and forward lowering departure, then a stop
supported on the arms.

A CH11 request now finishes the existing drive-reference ramps and hands wheel
control back through the existing stationary-PD slew. It retains the independent
500 ms calm requirement before arm preparation. This breaks the loop where the
neutral driving controller oscillated while waiting for calm before releasing.
Ordinary ground/standing driving and the successful fast lift remain unchanged.

Both arms already detected contact and started reversing in v6. The failure was
later support qualification: rocking briefly unloaded either arm and exceeded
the old +4°/s confirmation limit, keeping the wheels coasting. V7 begins the
bounded wheel stop once both independent contacts are detected and qualifies
support from per-arm observations within 60 ms, with 80 ms dwell and at least
80 ms of real elapsed time after both contacts. The qualification rate window is
−12..+12°/s; the global motion limits remain. A single impact cannot qualify.
Supported descent then continues arm return toward measured flat/Forward.

[Native and model evidence](../../../evidence/balance-lower/supported-return-v7/README.md):
255 → 258 completions across the same 329 model cases, three improvements and no
regressions. All 72 trial-informed contact cases complete and all 11 injected
failures reject completion. Fixed recorded-input replay now qualifies support,
but still rejects the old trace's later loss of support; new braking changes
that trajectory, so replay could not establish the physical result. The later
physical trial completed successfully; repeated trials are still needed to
establish reliability. New schema 7 metadata keeps the same 240-byte samples and
preserves historical exports.

After verified installation, use the existing [balance test procedure](../../BALANCE_TESTING.md).
Stand normally with CH6 LOW for a separate lowering trial, center the sticks,
then pulse CH11 once. Expect the drive stop to settle, forward arm placement,
contact followed by continued arm return, and a level/Forward finish. Disarm
both groups and archive the log before another run. Keep CH12 assistance out of
the recorded trial so the automatic sequence can be assessed.
