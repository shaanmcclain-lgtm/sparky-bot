#!/usr/bin/env python3
"""
SparkyBotMini Omni-Wheel Teleop - Clean Implementation
Hybrid control: Omni-directional movement (WASD) + Tank-drive rotation (JL)

Controls:
  W - forward (omni-drive)
  S - backward (omni-drive)
  A - strafe left (omni-drive)
  D - strafe right (omni-drive)
  J - rotate counter-clockwise (tank-drive)
  L - rotate clockwise (tank-drive)
  Space - stop (active braking)
  Q - quit
  +/- - adjust speed (20-100)
"""

import sys
import tty
import termios
import select
import time
from typing import Optional

# Import the SparkyBotMini library
# Assuming "Library or sumth" is importable as a module
try:
    from sparkybotmini import SparkyBotMini
except ImportError:
    print("Error: Could not import SparkyBotMini library")
    print("Make sure 'Library or sumth' is in your Python path")
    sys.exit(1)


# ============================================================================
# CONFIGURATION
# ============================================================================

# Speed settings
MIN_SPEED = 20
MAX_SPEED = 100
DEFAULT_SPEED = 60
SPEED_INCREMENT = 5

# Motor indices (assuming: m1=front_left, m2=front_right, m3=rear_left, m4=rear_right)
M1_FL = 0  # front_left
M2_FR = 1  # front_right
M3_RL = 2  # rear_left
M4_RR = 3  # rear_right

# Serial port configuration
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# Timing
KEY_HOLD_TIME = 0.1  # seconds - keeps motors active after key release
DEBUG = False


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def clamp(value: int, lo: int = -100, hi: int = 100) -> int:
    """Clamp value to motor range"""
    return max(lo, min(hi, int(value)))


def get_key(timeout: float = 0.05) -> Optional[str]:
    """Non-blocking keyboard input"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            # Handle escape sequences for arrow keys
            if ch == "\x1b":
                rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
                if rlist:
                    ch += sys.stdin.read(2)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ============================================================================
# OMNI-WHEEL DRIVE KINEMATICS
# ============================================================================

class OmniWheelDrive:
    """
    Omni-wheel drive controller for 4-wheel omni configuration.
    
    Wheel layout (top view):
      FL (1)        FR (2)
      
      RL (3)        RR (4)
    
    For omni-drive (WASD):
      - All wheels move together for translation
      - Forward: all wheels forward
      - Backward: all wheels backward
      - Left strafe: left wheels backward, right wheels forward
      - Right strafe: left wheels forward, right wheels backward
    
    For tank-drive rotation (JL):
      - Left and right sides move opposite
      - Counter-clockwise: left side backward, right side forward
      - Clockwise: left side forward, right side backward
    """
    
    def __init__(self, robot: SparkyBotMini, speed: int = DEFAULT_SPEED, debug: bool = False):
        self.robot = robot
        self.current_speed = clamp(speed, MIN_SPEED, MAX_SPEED)
        self.debug = debug
        self.last_motors = (0, 0, 0, 0)
    
    def set_speed(self, speed: int):
        """Update current speed"""
        self.current_speed = clamp(speed, MIN_SPEED, MAX_SPEED)
        if self.debug:
            print(f"[Speed: {self.current_speed}]")
    
    def stop(self):
        """Stop all motors (active braking)"""
        self.robot.set_motor(0, 0, 0, 0)
        self.last_motors = (0, 0, 0, 0)
        if self.debug:
            print("[STOP]")
    
    def forward(self):
        """Move forward (all wheels forward)"""
        s = self.current_speed
        motors = (s, s, s, s)
        self.robot.set_motor(*motors)
        self.last_motors = motors
        if self.debug:
            print(f"[FORWARD] {motors}")
    
    def backward(self):
        """Move backward (all wheels backward)"""
        s = -self.current_speed
        motors = (s, s, s, s)
        self.robot.set_motor(*motors)
        self.last_motors = motors
        if self.debug:
            print(f"[BACKWARD] {motors}")
    
    def strafe_left(self):
        """Strafe left (left wheels backward, right wheels forward)"""
        s = self.current_speed
        motors = (-s, s, -s, s)
        self.robot.set_motor(*motors)
        self.last_motors = motors
        if self.debug:
            print(f"[STRAFE LEFT] {motors}")
    
    def strafe_right(self):
        """Strafe right (left wheels forward, right wheels backward)"""
        s = self.current_speed
        motors = (s, -s, s, -s)
        self.robot.set_motor(*motors)
        self.last_motors = motors
        if self.debug:
            print(f"[STRAFE RIGHT] {motors}")
    
    def rotate_ccw(self):
        """Rotate counter-clockwise (tank-drive style)
        Left side backward, right side forward"""
        s = self.current_speed
        motors = (-s, s, -s, s)
        self.robot.set_motor(*motors)
        self.last_motors = motors
        if self.debug:
            print(f"[ROTATE CCW] {motors}")
    
    def rotate_cw(self):
        """Rotate clockwise (tank-drive style)
        Left side forward, right side backward"""
        s = self.current_speed
        motors = (s, -s, s, -s)
        self.robot.set_motor(*motors)
        self.last_motors = motors
        if self.debug:
            print(f"[ROTATE CW] {motors}")


# ============================================================================
# MAIN TELEOP LOOP
# ============================================================================

def teleop_main(port: str = SERIAL_PORT, debug: bool = False):
    """Main teleoperation loop"""
    
    # Initialize robot
    robot = SparkyBotMini(port=port, baudrate=BAUDRATE, debug=debug)
    
    if not robot.connect():
        print("ERROR: Failed to connect to robot on", port)
        return False
    
    print(f"✓ Connected to SparkyBotMini on {port}")
    
    # Create drive controller
    drive = OmniWheelDrive(robot, speed=DEFAULT_SPEED, debug=debug)
    
    # Print controls
    print("\n" + "="*50)
    print("  OMNI-WHEEL TELEOP CONTROLS")
    print("="*50)
    print("  MOVEMENT (Omni-Drive):")
    print("    W - Forward")
    print("    S - Backward")
    print("    A - Strafe Left")
    print("    D - Strafe Right")
    print("\n  ROTATION (Tank-Drive):")
    print("    J - Rotate Counter-Clockwise")
    print("    L - Rotate Clockwise")
    print("\n  UTILITY:")
    print("    + - Increase Speed")
    print("    - - Decrease Speed")
    print("    Space - Stop (Active Braking)")
    print("    Q - Quit")
    print("="*50)
    print(f"  Current Speed: {drive.current_speed}")
    print("="*50 + "\n")
    
    try:
        running = True
        last_command_time = time.time()
        last_key = None
        
        while running:
            key = get_key(timeout=0.05)
            now = time.time()
            
            # Apply active braking if no input for KEY_HOLD_TIME
            if not key:
                if now - last_command_time > KEY_HOLD_TIME:
                    drive.stop()
                continue
            
            key_lower = key.lower()
            
            # Handle speed adjustment
            if key_lower in ('+', '='):
                new_speed = drive.current_speed + SPEED_INCREMENT
                if new_speed <= MAX_SPEED:
                    drive.set_speed(new_speed)
                    print(f">>> Speed: {drive.current_speed}")
                continue
            
            elif key_lower == '-':
                new_speed = drive.current_speed - SPEED_INCREMENT
                if new_speed >= MIN_SPEED:
                    drive.set_speed(new_speed)
                    print(f">>> Speed: {drive.current_speed}")
                continue
            
            # Handle movement and rotation
            if key_lower == 'w':
                drive.forward()
            elif key_lower == 's':
                drive.backward()
            elif key_lower == 'a':
                drive.strafe_left()
            elif key_lower == 'd':
                drive.strafe_right()
            elif key_lower == 'j':
                drive.rotate_ccw()
            elif key_lower == 'l':
                drive.rotate_cw()
            elif key_lower == ' ':
                drive.stop()
            elif key_lower == 'q':
                print("\n>>> Quitting...")
                running = False
                break
            else:
                # Unknown key - ignore
                continue
            
            last_command_time = now
            last_key = key_lower
        
    except KeyboardInterrupt:
        print("\n>>> Interrupted by user")
    
    finally:
        # Cleanup
        print(">>> Stopping motors...")
        drive.stop()
        time.sleep(0.1)
        
        print(">>> Disconnecting...")
        robot.disconnect()
        print("✓ Disconnected\n")


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    # Parse command line arguments
    port = SERIAL_PORT
    debug = DEBUG
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    if len(sys.argv) > 2:
        debug = sys.argv[2].lower() in ('1', 'true', 'yes', 'debug')
    
    # Run teleoperation
    teleop_main(port=port, debug=debug)
