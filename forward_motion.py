#!/usr/bin/env python3
"""
Omni-wheel teleop for SparkyBotMini using keyboard controls.

Controls:
  W - forward
  S - backward
  A - strafe left
  D - strafe right
  J - rotate left (counter-clockwise)
  L - rotate right (clockwise)
  Space - stop
  Q - quit

This keeps using the SparkyBotMini API's set_motor(m1, m2, m3, m4) call.
Wheel order is assumed to be: front_left, front_right, rear_left, rear_right.
Motor values use a range of -100..100 (negative == reverse). If your
SparkyBotMini implementation uses a different convention, adjust the
mappings accordingly.
"""

import sys
import tty
import termios
import select
import time
from sparkybotmini import SparkyBotMini


def get_key(timeout=0.05):
    """Read a single keypress from stdin with timeout (non-blocking).
    Returns the character as a string or None if timeout.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            # handle arrow keys / escape sequences if needed
            if ch == "\x1b":
                # read remaining bytes if available
                rlist, _, _ = select.select([fd], [], [], 0.01)
                if rlist:
                    ch += sys.stdin.read(2)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def clamp(v, lo=-100, hi=100):
    return max(lo, min(hi, int(v)))


def teleop_omni(port="/dev/ttyUSB0", speed=80, debug=False):
    """Interactive teleoperation loop for omni wheels.

    - speed: base translational speed (0..100)
    - port: serial port for the SparkyBotMini
    """
    robot = SparkyBotMini(port=port, debug=debug)

    if not robot.connect():
        print("Failed to connect to robot")
        return

    print("Connected to SparkyBotMini on {}".format(port))
    print("Controls: W/A/S/D = move, J/L = rotate, Space = stop, Q = quit")
    print("Using wheel order: front_left, front_right, rear_left, rear_right")

    try:
        running = True

        # Stop at start
        robot.set_motor(0, 0, 0, 0)

        while running:
            key = get_key(0.1)
            if not key:
                # no keypress; continue
                continue

            k = key.lower()

            # default motors (stopped)
            fl = fr = rl = rr = 0

            if k == 'w':
                # forward
                fl = fr = rl = rr = speed
            elif k == 's':
                # backward
                fl = fr = rl = rr = -speed
            elif k == 'a':
                # strafe left (omni/mecanum style)
                # front_left: -speed, front_right: speed, rear_left: speed, rear_right: -speed
                fl = -speed
                fr = speed
                rl = speed
                rr = -speed
            elif k == 'd':
                # strafe right
                fl = speed
                fr = -speed
                rl = -speed
                rr = speed
            elif k == 'j':
                # rotate left (counter-clockwise)
                # left wheels backward, right wheels forward
                fl = -speed
                rl = -speed
                fr = speed
                rr = speed
            elif k == 'l':
                # rotate right (clockwise)
                fl = speed
                rl = speed
                fr = -speed
                rr = -speed
            elif k == ' ':
                # space: stop
                fl = fr = rl = rr = 0
            elif k == 'q':
                print("Quitting...")
                running = False
                break
            else:
                # unrecognized key: ignore
                continue

            # clamp and send
            fl, fr, rl, rr = map(clamp, (fl, fr, rl, rr))

            if debug:
                print(f"set_motor({fl}, {fr}, {rl}, {rr})")

            robot.set_motor(fl, fr, rl, rr)

    finally:
        # ensure motors are stopped and we disconnect
        try:
            robot.set_motor(0, 0, 0, 0)
        except Exception:
            pass
        robot.disconnect()
        print("Disconnected")


if __name__ == '__main__':
    # allow optional args: port, speed, debug
    port = '/dev/ttyUSB0'
    speed = 80
    debug = False

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            speed = int(sys.argv[2])
        except ValueError:
            pass
    if len(sys.argv) > 3:
        debug = sys.argv[3].lower() in ('1', 'true', 'yes')

    teleop_omni(port=port, speed=clamp(speed), debug=debug)
