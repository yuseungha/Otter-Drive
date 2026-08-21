"""Exact host-side copies of the active Arduino firmware contract.

Source: laptop_teleop/firmware/active_rc_car_controller_safe/
rc_car_controller_safe/rc_car_controller_safe.ino

Any change to the matching constants in that sketch must update this file in
the same commit. Host diagnostics treat this module as their single source.
"""

ADC_LEFT = 710
ADC_CENTER = 588
ADC_RIGHT = 466
STEER_DEADBAND = 10
STEER_MIN_PWM = 110
STEER_MAX_PWM = 180
WATCHDOG_SEC = 0.400
DEBUG_SEC = 0.200
ESC_ARM_SEC = 3.0


def arduino_map(value, in_min, in_max, out_min, out_max):
    """Match Arduino map() integer division, including truncation toward zero."""
    numerator = (value - in_min) * (out_max - out_min)
    return int(numerator / (in_max - in_min)) + out_min


def steering_command_to_adc(command):
    """Match steeringCommandToAdc() in the active firmware."""
    command = max(-1000, min(1000, int(command)))
    if command >= 0:
        return arduino_map(command, 0, 1000, ADC_CENTER, ADC_LEFT)
    return arduino_map(command, -1000, 0, ADC_RIGHT, ADC_CENTER)
