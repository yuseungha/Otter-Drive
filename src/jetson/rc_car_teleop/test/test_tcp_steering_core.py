"""Unit tests for the steering-only wire protocol."""

import pytest

from rc_car_teleop.tcp_steering_core import (
    decode_steering_packet,
    encode_steering_packet,
    select_forwarded_steering,
)


def test_round_trip() -> None:
    assert decode_steering_packet(encode_steering_packet(7, -123)) == (7, -123)


def test_forwarding_requires_transport_and_serial_readiness() -> None:
    assert select_forwarded_steering(
        240, connected=True, fresh=True, serial_ready=True) == 240
    assert select_forwarded_steering(
        240, connected=False, fresh=True, serial_ready=True) == 0
    assert select_forwarded_steering(
        240, connected=True, fresh=False, serial_ready=True) == 0
    assert select_forwarded_steering(
        240, connected=True, fresh=True, serial_ready=False) == 0


@pytest.mark.parametrize('steering', [-601, 601, 1000])
def test_rejects_out_of_range_steering(steering: int) -> None:
    with pytest.raises(ValueError):
        decode_steering_packet(encode_steering_packet(1, steering))


@pytest.mark.parametrize('line', [
    b'not-json',
    b'{"v":2,"seq":1,"steering":0}',
    b'{"v":1,"seq":-1,"steering":0}',
    b'{"v":1,"seq":true,"steering":0}',
])
def test_rejects_invalid_packet(line: bytes) -> None:
    with pytest.raises(ValueError):
        decode_steering_packet(line)
