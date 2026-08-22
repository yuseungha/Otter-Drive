# TRX-4 LOW/HIGH gear servo setup

## Wiring

The photographed servo lead uses the common JR/Futaba color order:

| Servo wire | Function | Connection |
| --- | --- | --- |
| Orange | Control signal | Arduino Uno D10 |
| Red | Servo power | Regulated 5-6 V BEC/receiver positive |
| Brown | Ground | BEC/receiver ground and Arduino GND |

Do not power the shift servo from the Uno 5 V pin. Do not leave its signal
wire connected to both the Traxxas receiver and D10 at the same time.

## Keyboard controls

- `1`: request LOW gear
- `2`: request HIGH gear
- `Space`: set throttle to zero

The Uno applies a new gear only while throttle is zero. If a gear key is
pressed while moving, the request remains pending and is applied after
`Space` brings the requested throttle to zero.

## First calibration

The servo model and travel are unknown. The firmware therefore starts with
unverified bench values:

- `GEAR_LOW_US = 1300`
- `GEAR_HIGH_US = 1700`

Before ground driving:

1. Raise all wheels and disconnect motor power.
2. Disconnect the shift linkage from the servo horn. Do not turn the servo
   output shaft by hand.
3. Power the servo from the regulated BEC and keep Arduino/BEC grounds common.
4. With throttle at zero, press `1` and `2` and observe the two positions.
5. If LOW/HIGH are reversed, swap `GEAR_LOW_US` and `GEAR_HIGH_US`.
6. If either position is short of engagement or the servo buzzes at an end,
   adjust that pulse in 20 us increments and retest with the linkage detached.
7. Reconnect the linkage only after both endpoints move freely without binding.
8. Compare LOW/HIGH at the same small throttle command before increasing speed.

The pulse values are configuration starting points, not confirmed endpoints for
the unidentified servo.
