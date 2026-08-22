from rc_car_teleop.autonomous_drive_core import (
    AutonomousDriveConfig,
    convert_command,
)


def test_zero_and_nonfinite_fail_closed():
    config = AutonomousDriveConfig()
    assert convert_command(0.0, 0.0, config) == (0, 0)
    assert convert_command(float('nan'), 0.0, config) == (0, 0)


def test_si_command_is_bounded_to_counts():
    config = AutonomousDriveConfig(
        throttle_max=300, steering_max=900, steering_sign=-1)
    assert convert_command(1.0, 2.0, config) == (300, -900)
    assert convert_command(0.075, -0.6108652382, config) == (150, 900)
