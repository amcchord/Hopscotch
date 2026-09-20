# V9 bounded support confirmation

The [v8 recording](../trial-v8-stop-20260920/README.md) contacts both arms but
repeatedly loses confirmation during a small loaded rebound. V9 extends
only that confirmation path to +20 degrees/s and 1.5 degrees above the
impact minimum within the existing 300 ms impact window. The ordinary
+12 limit remains outside that exception. Both recent independent support
observations, arm velocity bounds and the real 80 ms dwell remain required.
Wheel braking, initial fall, 0.24-rad/s supported return, motor cap, final
retraction and all global/support-loss/time/progress guards are unchanged.

Native tests reproduce the six recorded contact frames and independently
reject missing support, excessive rate, excessive rise, a late rebound,
fast arms and arms still moving outward. Existing single-impulse and
late-second-contact guards remain covered.

Paired screen: `python3 scripts/simulate_lowering.py --baseline-ref 47eb19a
--output output/lowering-v9-screen`. [Summary](summary.json),
[comparison](comparison.json), [candidate](simulation.json),
[baseline](baseline-simulation.json), [nominal trace](nominal.csv),
[rebound trace](rebound-stress.csv).

All 329 outcomes are unchanged: 258 complete, all 72 trial-informed contact
cases complete, 15/24 delayed-contact cases complete, and 11 injected faults
reject. The approximate model does not reproduce the exact recorded
transient and does not establish hardware reliability. Recorded replay
locates command divergence; manual acceptance is still needed.
