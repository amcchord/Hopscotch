# First corrected-firmware attempt

Firmware: source b5be4ef, application SHA-256 46e77224ad774204625c8878574f8581b40fe0547fb4cb6e53d1a7cabe6b9608.

Austin reports that the robot stood up, started to run away, needed a hand intervention, then stabilized and remained balanced vertically. The exact intervention time was not marked. This is a hand-assisted recovery, not an unaided stand-up success.

Serial evidence confirms both rear motors entered Speed mode, TIPPING UP and BALANCING were reached, and the arms returned. The 120-second recorder stopped with 5,648 rows because of its duration limit while the controller remained in BALANCE. The later end/disarm is outside that data window. No tool commanded arming or motion.

The follow-up question asks Austin to support the robot, lower CH9 and CH10 and leave power/USB connected for saving and download.
