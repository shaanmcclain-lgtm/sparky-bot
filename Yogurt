#!/usr/bin/env python3
"""
Simple forward motion controller for X-configured omni wheel robot.
500 RPM motors at full speed.
"""

from motor_driver import MotorDriver
from encoder_reader import EncoderReader

class SimpleForwardMotion:
    def __init__(self, wheel_radius_m: float = 0.05):
        """
        Initialize forward motion controller.
        
        For 500 RPM motors:
        - RPM to rad/s: 500 * 2π / 60 ≈ 52.36 rad/s
        - Linear speed: wheel_radius * angular_velocity
        """
        self.motor_driver = MotorDriver()
        self.encoder_reader = EncoderReader()
        
        # 500 RPM = 52.36 rad/s
        self.max_rpm = 500
        self.max_rad_per_s = (self.max_rpm * 2 * 3.14159) / 60
        self.wheel_radius = wheel_radius_m
        
        # For X-configured omni wheels going forward:
        # All wheels spin in same direction at same speed
        self.forward_speed_m_per_s = self.wheel_radius * self.max_rad_per_s
    
    def go_forward(self, speed_percent: float = 1.0) -> bool:
        """
        Drive robot forward at specified speed.
        
        Args:
            speed_percent: 0.0 to 1.0 (1.0 = max speed)
        
        Returns:
            True if command sent successfully
        """
        # For X-configuration omni wheels, all 4 wheels need same speed
        speed = self.forward_speed_m_per_s * speed_percent
        
        # Set both left and right wheels to same speed
        return self.motor_driver.set_wheel_speeds(
            left_m_per_s=speed,
            right_m_per_s=speed
        )
    
    def stop(self) -> None:
        """Stop the robot immediately."""
        self.motor_driver.emergency_stop()
    
    def coast(self) -> None:
        """Coast to a stop (zero speed, no emergency)."""
        self.motor_driver.set_wheel_speeds(0.0, 0.0)


# Example usage
if __name__ == "__main__":
    motion = SimpleForwardMotion(wheel_radius_m=0.05)
    
    # Go forward at full speed
    print("Going forward at 100% speed...")
    motion.go_forward(speed_percent=1.0)
    
    # Check battery
    print(f"Battery voltage: {motion.motor_driver.get_battery_voltage()}V")
    
    # Stop after some time (caller would add timing logic)
    motion.stop()
