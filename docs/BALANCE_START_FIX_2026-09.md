# September 13, 2026 — CH11 received, stand-up refused

The transmitter was working. The September candidate introduced a startup regression: it required acceleration-limit readback from the RS05 rear drive motors, whose acceleration register is write-only. This prevented the stand-up sequence from reaching any arm movement. The correction checks acceleration-write transmission and keeps mandatory current-limit readback.

## What the robot showed

During Austin's single CH11 press, firmware printed `Single tap Ch11 -- normal tip-up`, attempted acceleration setup on rear motor IDs 30 and 20 four times each, then printed `Tip-up REFUSED: speed mode switch failed`. Both motor groups were armed, calibration and IMU were healthy, and no pending log save blocked the request. The refusal occurred before logging or the tip-up arm command. See [trigger capture](../evidence/balance-start-diagnosis/trigger-observation.serial) and [initial status](../evidence/balance-start-diagnosis/initial-status.serial).

After reconnecting USB with both groups disarmed, read-only probes returned:

| Motor | Current limit, `0x7018` | Acceleration, `0x7022` |
|---|---:|---:|
| Rear left, ID 30 | 10 A | 0 |
| Rear right, ID 20 | 10 A | 0 |

These acceleration zeros are actual returned values in the later explicit probe. They do **not** establish that the applied acceleration is zero. The original retry message also printed zero when a read timed out, so the trigger capture alone could not distinguish those two cases. [Probe evidence](../evidence/balance-start-diagnosis/reconnect-register-check.serial).

## Motor-specific protocol finding

The manufacturer's [RS05 manual, revision 260713](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS05/RS05User%20Manual260713.pdf), printed page 52 / PDF page 54, marks `0x7022 acc_rad` as **W**, while readable parameters are marked **W/R** or **R**. Its velocity-mode instructions use an acceleration write. The parameter table was extracted and visually checked. Readback cannot be used to confirm this RS05 setting.

The project's shared protocol header was based on the RS00 manual, and Hopscotch uses RS05 drive motors with RS00 arms. Applying the RS00 readback assumption to the drive motors was incorrect. Older firmware attempted the same acceleration read but ignored the failure; the September change propagated that failure and made every ordinary stand-up refuse.

## Firmware correction

- `MotorManager` checks the CAN transmit result, retries a failed transmission up to four attempts, and skips unsupported readback specifically for `ACC_RAD`. Other checked parameters continue to require matching readback.
- Current-limit verification remains required on both rear motors. Missing, mismatched or nonfinite replies still refuse setup and request a motor stop.
- Diagnostics distinguish a failed transmission, a missing reply and a mismatched value. The acceleration constant now explicitly records RS05 write-only access.
- Balance gains, trim, arm calibration, channel mapping, sensor checks and motion limits are unchanged by this correction.

A successful acceleration write means the controller accepted the frame for transmission. It is not proof that a particular motor applied the value; the motor does not expose that verification through this register. The follow-up physical capture remains necessary.

## Verification and release

The new native test runs the actual production `MotorManager` against a simulated RS05 transport. It fails against the previous code at the ordinary Speed-mode transition and passes with the correction. It covers zero or missing acceleration readback, both rear motors, transmission retries and exhausted retries, unarmed refusal, and mandatory current readback with missing, mismatched and nonfinite replies. All test arming is simulated; no test accesses hardware.

The consolidated check also runs existing native control/USB checks, twelve Python tests, syntax/whitespace checks and the ESP32-S3 firmware build. [Validation output](../evidence/balance-start-diagnosis/validation.txt). The corrected application is packaged separately under `artifacts/balance-start-fix/`; the previous package and full original device backup remain available.

Release and live verification results are recorded in [current state](progress/CURRENT.md). A successful upload or disarmed check does not demonstrate physical balance reliability.
