#!/usr/bin/env python3
"""PTY-backed model of the active RC-car Arduino firmware."""

import argparse
import os
import pty
import re
import select
import time

from rc_car_teleop.firmware_model import (
    ADC_CENTER,
    DEBUG_SEC,
    ESC_ARM_SEC,
    STEER_DEADBAND,
    STEER_MAX_PWM,
    STEER_MIN_PWM,
    WATCHDOG_SEC,
    steering_command_to_adc,
)


class FakeArduino:
    """Minimal time-stepped serial model used by integration tests."""

    def __init__(self, master_fd, clock=time.monotonic):
        self.master_fd = master_fd
        self.clock = clock
        self.started_at = clock()
        self.last_debug_at = self.started_at
        self.last_command_at = None
        self.target_adc = ADC_CENTER
        self.current_adc = float(ADC_CENTER)
        self.throttle = 0
        self.gear = -1
        self.drive_active = False
        self.steering_enabled = False
        self.esc_armed = False
        self.buffer = bytearray()

    def write_line(self, text):
        os.write(self.master_fd, (text + '\r\n').encode('ascii'))

    def boot(self):
        self.write_line('====================================')
        self.write_line('RC car controller started')
        self.write_line('Command: D throttle steering gear')

    def process_line(self, line, now):
        match = re.fullmatch(
            r'D\s*(-?\d+)\s+(-?\d+)(?:\s+(-?\d+))?', line)
        if match:
            try:
                throttle = int(match.group(1))
                steering = int(match.group(2))
                gear = (
                    self.gear if match.group(3) is None
                    else int(match.group(3)))
            except ValueError:
                self.write_line('[ERROR] Use: D throttle steering gear / X / R')
                return
            if gear not in (-1, 1):
                self.write_line('[ERROR] Gear must be -1 (LOW) or 1 (HIGH)')
                return
            if not -1000 <= throttle <= 1000:
                self.throttle = 0
                self.steering_enabled = False
                self.drive_active = False
                self.write_line(
                    '[ERROR] Throttle must be -1000..1000; outputs stopped')
                return
            self.throttle = throttle
            self.target_adc = steering_command_to_adc(steering)
            self.gear = gear
            self.steering_enabled = True
            self.drive_active = True
            self.last_command_at = now
            label = 'LOW' if gear == -1 else 'HIGH'
            self.write_line(
                f'[COMMAND] throttle={throttle} steeringADC='
                f'{self.target_adc} gear={label}')
            return
        if line == 'X':
            self.throttle = 0
            self.steering_enabled = False
            self.drive_active = False
            self.write_line('[STOP]')
            return
        if line == 'R':
            self.steering_enabled = False
            self.target_adc = int(round(self.current_adc))
            self.write_line('[FAULT RESET]')
            return
        self.write_line('[ERROR] Use: D throttle steering gear / X / R')

    def _read_commands(self, now):
        readable, _, _ = select.select([self.master_fd], [], [], 0)
        if readable:
            try:
                self.buffer.extend(os.read(self.master_fd, 4096))
            except OSError:
                return
        while b'\n' in self.buffer:
            raw, _, remainder = self.buffer.partition(b'\n')
            self.buffer = bytearray(remainder)
            line = raw.rstrip(b'\r').decode('ascii', errors='replace').strip()
            if line:
                self.process_line(line, now)

    def _move_steering(self, dt):
        error = self.target_adc - self.current_adc
        if not self.steering_enabled or abs(error) <= STEER_DEADBAND:
            return
        pwm = max(STEER_MIN_PWM, min(STEER_MAX_PWM, 110 + abs(error)))
        counts_per_sec = 45.0 + 0.55 * pwm
        step = min(abs(error), counts_per_sec * dt)
        self.current_adc += step if error > 0 else -step

    def tick(self, now, dt):
        self._read_commands(now)
        if (
            self.drive_active and self.last_command_at is not None
            and now - self.last_command_at > WATCHDOG_SEC
        ):
            self.throttle = 0
            self.steering_enabled = False
            self.drive_active = False
            self.write_line('[WATCHDOG] Command timeout: outputs stopped')
        if not self.esc_armed and now - self.started_at >= ESC_ARM_SEC:
            self.esc_armed = True
            self.write_line('[ESC ARMED]')
        self._move_steering(dt)
        if now - self.last_debug_at >= DEBUG_SEC:
            self.last_debug_at = now
            current = int(round(self.current_adc))
            error = self.target_adc - current
            enabled = 'YES' if self.steering_enabled else 'NO'
            drive = 'STOP' if abs(error) <= STEER_DEADBAND else (
                'ADC_UP' if error > 0 else 'ADC_DOWN')
            pwm = 0 if drive == 'STOP' else max(
                STEER_MIN_PWM, min(STEER_MAX_PWM, 110 + abs(error)))
            self.write_line(
                f'[DEBUG] Current={current} Target={self.target_adc} '
                f'Error={error} PWM={pwm} Drive={drive} '
                f'Enabled={enabled} Fault=NO')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tick-sec', type=float, default=0.01)
    args = parser.parse_args()
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    print(slave_path, flush=True)
    model = FakeArduino(master_fd)
    model.boot()
    previous = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            model.tick(now, now - previous)
            previous = now
            time.sleep(args.tick_sec)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == '__main__':
    main()
