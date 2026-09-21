# CH6 fast laydown: low-torque contact correction

Use **CH6 HIGH, then pulse CH11** for fast laydown. LOW/center retains normal
speed; stand-up is unchanged. Check [installed identity](progress/CURRENT.md).

V12 could stop after catching because low holding torque was treated as loss
of support. V13 continues the established slow arm return when those readings
are low and the body is calm; it can accelerate when load is measured. A rapid
fall without measured support still pauses, and all existing fault/landing
checks remain. [Failure and verification](../evidence/balance-lower/fast-return-v13/README.md).

For one manual trial, let standing settle, center CH1/CH2, select CH6 HIGH and
pulse CH11 once. Observe continuous return to flat with arms Forward. Disarm
both groups and leave power on for archive before another run. Follow the
existing [test procedure](BALANCE_TESTING.md). Physical speed is not guaranteed
to be three times faster; completing the return takes priority over acceleration.
