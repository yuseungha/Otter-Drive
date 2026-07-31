#!/usr/bin/env python3
"""Pulse USB serial DTR once so an Arduino Uno enters its bootloader."""

import time

import serial


PORT = "/dev/ttyACM0"


with serial.Serial(PORT, 115200, timeout=0.1) as connection:
    connection.dtr = False
    time.sleep(0.20)
    connection.dtr = True
    time.sleep(0.10)

time.sleep(0.20)

