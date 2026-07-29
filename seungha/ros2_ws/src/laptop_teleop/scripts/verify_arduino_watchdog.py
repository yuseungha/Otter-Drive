#!/usr/bin/env python3
"""Verify Arduino throttle clamp and command watchdog with motor power off."""

import argparse
import sys
import time

import serial


def read_for(connection: serial.Serial, duration_sec: float) -> str:
    deadline = time.monotonic() + duration_sec
    received = bytearray()
    while time.monotonic() < deadline:
        waiting = connection.in_waiting
        if waiting:
            received.extend(connection.read(waiting))
        time.sleep(0.02)
    return received.decode('utf-8', errors='replace')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.05) as connection:
        # Opening an Uno serial port resets it. Allow boot and ESC arming time.
        time.sleep(3.6)
        connection.reset_input_buffer()
        connection.write(b'D 0 0 -1\n')
        connection.flush()
        time.sleep(0.10)
        # Deliberately exceed the configured throttle limit. Firmware must
        # clamp it while the motor power is disconnected.
        connection.write(b'D 1200 0 -1\n')
        connection.flush()
        output = read_for(connection, 1.20)
        connection.write(b'X\n')
        connection.flush()

    print(output, end='')
    clamp_ok = '[COMMAND] throttle=1050' in output
    watchdog_ok = '[WATCHDOG] Command timeout: outputs stopped' in output
    if not clamp_ok:
        print('FAIL: throttle clamp was not observed', file=sys.stderr)
    if not watchdog_ok:
        print('FAIL: watchdog stop was not observed', file=sys.stderr)
    if clamp_ok and watchdog_ok:
        print('PASS: throttle clamp and watchdog verified')
        return 0
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
