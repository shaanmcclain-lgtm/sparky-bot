#!/usr/bin/env python3
"""
Omni-wheel teleop for SparkyBotMini using keyboard controls with motor tuning.

Controls:
  W - forward
  S - backward
  A - strafe left
  D - strafe right
  J - rotate left (counter-clockwise)
  L - rotate right (clockwise)
  X - execute square sequence (forward 2s, turn 90°, repeat 3x)
  Space - stop (active braking)
  Q - quit

MOTOR TUNING:
  Adjust the power multipliers below if certain motors run faster/slower.
  Each motor can be tuned independently (0.0 to 1.0 scale):
  - MOTOR_FL_POWER: front_left motor power
  - MOTOR_FR_POWER: front_right motor power
  - MOTOR_RL_POWER: rear_left motor power
  - MOTOR_RR_POWER: rear_right motor power

ACTIVE BRAKING:
  When no key is pressed, motors are immediately set to 0 (active braking).
  This replaces the previous "continue" behavior that kept momentum.

This uses the SparkyBotMini API's set_motor(m1, m2, m3, m4) call.
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

# ============================================================================
# MOTOR TUNING SECTION
# ============================================================================
# Adjust these values if motors run at different speeds.
# Valid range: 0.0 to 1.0 (where 1.0 = full power)
# If front_left is slower, increase MOTOR_FL_POWER
# If rear_right is faster, decrease MOTOR_RR_POWER
# ============================================================================
MOTOR_FL_POWER = 1.0   # front_left power multiplier
MOTOR_FR_POWER = 1.0   # front_right power multiplier
MOTOR_RL_POWER = 1.0   # rear_left power multiplier
MOTOR_RR_POWER = 1.0   # rear_right power multiplier

# Sequence timing parameters
SEQUENCE_FORWARD_TIME = 2.0   # seconds to move forward
SEQUENCE_TURN_TIME = 1.0      # approximate time for 90-degree turn (tuning needed)
SEQUENCE_SPEED = 100          # speed for sequence moves (0-100)
SEQUENCE_TURN_SPEED = 80      # speed for sequence turns (0-100)


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


def apply_motor_tuning(fl, fr, rl, rr):
    """Apply motor power multipliers for tuning.
    
    Args:
        fl, fr, rl, rr: raw motor commands (-100 to 100)
    
    Returns:
        Tuple of tuned motor commands
    """
    fl_tuned = clamp(fl * MOTOR_FL_POWER)
    fr_tuned = clamp(fr * MOTOR_FR_POWER)
    rl_tuned = clamp(rl * MOTOR_RL_POWER)
    rr_tuned = clamp(rr * MOTOR_RR_POWER)
    return fl_tuned, fr_tuned, rl_tuned, rr_tuned


def execute_square_sequence(robot, speed, turn_speed, debug=False):
    """Execute a square pattern: forward 2s, turn 90°, repeat 3x.
    
    Args:
        robot: SparkyBotMini robot instance
        speed: forward speed (0-100)
        turn_speed: turning speed (0-100)
        debug: if True, print debug info
    """
    print("\n=== Starting Square Sequence ===")
    print(f"Forward speed: {speed}, Turn speed: {turn_speed}")
    
    try:
        # Repeat 3 times (moves forward, turns 90°)
        for i in range(3):
            print(f"\nIteration {i+1}/3:")
            
            # Move forward for SEQUENCE_FORWARD_TIME seconds
            print(f"  Moving forward for {SEQUENCE_FORWARD_TIME} sec...")
            fl, fr, rl, rr = speed, speed, speed, speed
            fl, fr, rl, rr = apply_motor_tuning(fl, fr, rl, rr)
            robot.set_motor(fl, fr, rl, rr)
            time.sleep(SEQUENCE_FORWARD_TIME)
            
            # Stop (active braking)
            print("  Stopping (active braking)...")
            robot.set_motor(0, 0, 0, 0)
            time.sleep(0.2)
            
            # Turn 90 degrees clockwise (left wheels forward, right wheels backward)
            print(f"  Turning 90° for {SEQUENCE_TURN_TIME} sec...")
            fl, fr, rl, rr = turn_speed, -turn_speed, turn_speed, -turn_speed
            fl, fr, rl, rr = apply_motor_tuning(fl, fr, rl, rr)
            robot.set_motor(fl, fr, rl, rr)
            time.sleep(SEQUENCE_TURN_TIME)
            
            # Stop (active braking)
            print("  Stopping (active braking)...")
            robot.set_motor(0, 0, 0, 0)
            time.sleep(0.2)
        
        # Final forward move (completes the square loop)
        print(f"\nFinal forward move for {SEQUENCE_FORWARD_TIME} sec...")
        fl, fr, rl, rr = speed, speed, speed, speed
        fl, fr, rl, rr = apply_motor_tuning(fl, fr, rl, rr)
        robot.set_motor(fl, fr, rl, rr)
        time.sleep(SEQUENCE_FORWARD_TIME)
        
        # Final stop
        print("Stopping (active braking)...")
        robot.set_motor(0, 0, 0, 0)
        
        print("=== Square Sequence Complete ===\n")
        
    except Exception as e:
        print(f"Error during sequence: {e}")
        robot.set_motor(0, 0, 0, 0)


def teleop_omni(port="/dev/ttyUSB0", speed=80, debug=False):
    """Interactive teleoperation loop for omni wheels with motor tuning.

    - speed: base translational speed (0..100)
    - port: serial port for the SparkyBotMini
    - debug: enable debug output
    """
    robot = SparkyBotMini(port=port, debug=debug)

    if not robot.connect():
        print("Failed to connect to robot")
        return

    print("Connected to SparkyBotMini on {}".format(port))
    print("\n=== CONTROLS ===")
    print("  W/A/S/D = move (forward/strafe-left/backward/strafe-right)")
    print("  J/L = rotate (counter-clockwise/clockwise)")
    print("  X = execute square sequence (2s forward, 90° turn, repeat)")
    print("  Space = stop (active braking)")
    print("  Q = quit")
    print("\n=== MOTOR TUNING ===")
    print(f"  FL power: {MOTOR_FL_POWER}, FR power: {MOTOR_FR_POWER}")
    print(f"  RL power: {MOTOR_RL_POWER}, RR power: {MOTOR_RR_POWER}")
    print("  (Edit the constants at the top of this script to adjust)\n")
    print("Using wheel order: front_left, front_right, rear_left, rear_right")

    try:
        running = True

        # Stop at start
        robot.set_motor(0, 0, 0, 0)

        while running:
            key = get_key(0.1)
            
            # If no key pressed, apply active braking (motors to 0)
            if not key:
                robot.set_motor(0, 0, 0, 0)
                continue

            k = key.lower()

            # default motors (stopped) - will be overridden if key matched
            fl = fr = rl = rr = 0
            is_movement_command = False

            if k == 'w':
                # forward
                fl = fr = rl = rr = speed
                is_movement_command = True
            elif k == 's':
                # backward
                fl = fr = rl = rr = -speed
                is_movement_command = True
            elif k == 'a':
                # strafe left (omni/mecanum style)
                fl = -speed
                fr = speed
                rl = speed
                rr = -speed
                is_movement_command = True
            elif k == 'd':
                # strafe right
                fl = speed
                fr = -speed
                rl = -speed
                rr = speed
                is_movement_command = True
            elif k == 'j':
                # rotate left (counter-clockwise)
                fl = -speed
                rl = -speed
                fr = speed
                rr = speed
                is_movement_command = True
            elif k == 'l':
                # rotate right (clockwise)
                fl = speed
                rl = speed
                fr = -speed
                rr = -speed
                is_movement_command = True
            elif k == 'x':
                # execute square sequence
                execute_square_sequence(robot, SEQUENCE_SPEED, SEQUENCE_TURN_SPEED, debug)
                continue
            elif k == ' ':
                # space: explicit stop (active braking)
                fl = fr = rl = rr = 0
                is_movement_command = True
            elif k == 'q':
                print("Quitting...")
                running = False
                break
            else:
                # unrecognized key: ignore (motors remain at 0 due to active braking)
                continue

            # Apply motor tuning and send
            fl, fr, rl, rr = apply_motor_tuning(fl, fr, rl, rr)

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
