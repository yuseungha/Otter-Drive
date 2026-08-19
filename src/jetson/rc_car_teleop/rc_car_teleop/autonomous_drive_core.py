"""Pure conversion from SI cone-controller commands to Arduino counts."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class AutonomousDriveConfig:
    max_speed_mps: float = 0.15
    max_steering_angle_rad: float = 0.6108652382
    throttle_max: int = 300
    steering_max: int = 900
    steering_sign: int = 1
    speed_deadband_mps: float = 0.01

    def validate(self) -> None:
        if self.max_speed_mps <= 0.0 or self.max_steering_angle_rad <= 0.0:
            raise ValueError('SI limits must be positive')
        if self.throttle_max < 0 or self.steering_max < 0:
            raise ValueError('count limits must be nonnegative')
        if self.steering_sign not in {-1, 1}:
            raise ValueError('steering_sign must be -1 or 1')


def convert_command(
    speed_mps: float, steering_angle_rad: float,
    config: AutonomousDriveConfig,
) -> tuple[int, int]:
    config.validate()
    if not isfinite(speed_mps) or not isfinite(steering_angle_rad):
        return 0, 0
    if speed_mps < config.speed_deadband_mps or speed_mps < 0.0:
        throttle = 0
    else:
        throttle = round(min(speed_mps, config.max_speed_mps)
                         / config.max_speed_mps * config.throttle_max)
    steering = round(
        max(-config.max_steering_angle_rad,
            min(config.max_steering_angle_rad, steering_angle_rad))
        / config.max_steering_angle_rad * config.steering_max
        * config.steering_sign)
    return int(throttle), int(steering)
