"""Small, testable protocol helpers for laptop-to-Jetson steering."""

from __future__ import annotations

import json


PROTOCOL_VERSION = 1


def select_forwarded_steering(
    steering: int,
    *,
    connected: bool,
    fresh: bool,
    serial_ready: bool,
) -> int:
    """Return steering only after transport and Arduino gates are ready."""
    if not connected or not fresh or not serial_ready:
        return 0
    return int(steering)


def encode_steering_packet(sequence: int, steering: int) -> bytes:
    """Encode one newline-delimited steering-only packet."""
    payload = {
        'v': PROTOCOL_VERSION,
        'seq': int(sequence),
        'steering': int(steering),
    }
    return (json.dumps(payload, separators=(',', ':')) + '\n').encode('ascii')


def decode_steering_packet(
    line: bytes,
    max_abs_steering: int = 600,
) -> tuple[int, int]:
    """Validate one packet and return ``(sequence, steering)``."""
    if len(line) > 256:
        raise ValueError('packet is too long')
    try:
        payload = json.loads(line.decode('ascii'))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError('packet is not valid ASCII JSON') from error
    if not isinstance(payload, dict):
        raise ValueError('packet must be an object')
    if payload.get('v') != PROTOCOL_VERSION:
        raise ValueError('unsupported protocol version')
    sequence = payload.get('seq')
    steering = payload.get('steering')
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise ValueError('sequence must be an integer')
    if isinstance(steering, bool) or not isinstance(steering, int):
        raise ValueError('steering must be an integer')
    if sequence < 0:
        raise ValueError('sequence must be non-negative')
    maximum = int(max_abs_steering)
    if maximum <= 0 or abs(steering) > maximum:
        raise ValueError('steering is outside the allowed range')
    return sequence, steering
