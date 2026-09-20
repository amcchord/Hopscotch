# Supported return v7 — scope and offline checks

The [physical v6 run](../trial-v6-20260920/README.md) revealed two ownership gates:
neutral driving stayed active while oscillation prevented its calm dwell, and
rocking arm contact never qualified the later supported descent.

V7 changes only explicit lowering requests. Once the existing pilot velocity
and turn ramps reach zero, the controller hands back through the existing
BalanceDrive slew to stationary PD. Ordinary stick driving and its stop contract
are unchanged. The independent 500 ms lower-preparation calm gate remains.

After two independent arm contacts, wheel speed starts approaching zero at the
existing 3 rad/s² bound. Support confirmation accepts each arm's load observed
within 60 ms over an 80 ms dwell, and requires at least 80 ms of real elapsed
time since BOTH first contacts. Body rate must be −12..+12°/s. This recognizes
brief rocking/unloading while rejecting one-off or one-arm impacts. The global
65°/s, wheel6rad/s, feedback/owner100ms, torque thresholds, arm speeds/lead,
support-loss/deadline checks and measured flat/Forward completion remain.

Native regressions cover bounded reference stopping and actual BalanceDrive
handoff, unchanged ordinary behavior, the measured contact sequence, continued
supported return, single stationary-arm impulses, delayed second-arm impulses,
clock rollover and existing failure cases. The fast owner independently checked
the single-impact edge and scoped handoff wiring without editing this checkout.

[329-case paired model](simulation.json) against [installed v6](baseline-simulation.json):
255 → 258 completions, [three improvements and no regressions](comparison.json).
All 72 trial-informed contact cases complete; delayed contact remains 15/24.
All 11 injected failures reject completion. [Summary](summary.json). The model
does not represent the earlier standing-drive handoff, lateral dynamics or
identified contact/geometry; it stops at faults. Sampled guards do not bound
all between-tick peaks. This remains an experimental physical-test candidate.
Reproduce with `python3 scripts/simulate_lowering.py --baseline-ref a772ecc`.

Recorded sensor replay reproduces v6's wrong-direction fault. V7 recognizes
support but later rejects the old trace's renewed loss of support. Candidate
commands diverge before that event, so this is neither a physical success nor
failure prediction. The second physical trial must establish the outcome.
