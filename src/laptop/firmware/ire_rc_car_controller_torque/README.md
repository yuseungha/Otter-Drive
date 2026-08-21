# IRE steering torque firmware

This sketch is based on the byte-verified 2026-08-13 competition firmware.
It changes only the steering minimum PWM and the boot identity:

- STEER_MIN_PWM: 110 to 140 (raised in 10-count live-test steps)
- STEER_MAX_PWM: unchanged at 180
- boot identity: IRE_TORQUE_20260820...PWM140_180

The higher minimum raises low- and medium-error steering effort without
raising the endpoint peak. Flash only through the persistent CH340 by-id path,
then verify the exact boot identity and a single X/[STOP] exchange before
enabling drive output.
