#!/usr/bin/env python3
"""
Bare-bones API for SparkyBot - move forward for 5 seconds.
"""

import time
from sparkybotmini import SparkyBotMini


def move_forward_5_seconds():
    """Move robot forward for 5 seconds."""
    robot = SparkyBotMini(port="/dev/ttyUSB0", debug=False)
    
    if not robot.connect():
        print("Failed to connect to robot")
        return
    
    try:
        print("Starting forward motion...")
        robot.set_motor(100, 100, 100, 100)  # Full speed forward
        
        time.sleep(5)
        
        print("Stopping...")
        robot.set_motor(0, 0, 0, 0)  # Stop
        
    finally:
        robot.disconnect()


if __name__ == "__main__":
    move_forward_5_seconds()
