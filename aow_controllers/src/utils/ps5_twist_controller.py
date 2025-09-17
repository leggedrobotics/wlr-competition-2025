#!/usr/bin/env python3
"""
Simple PS5 controller to twist message converter for AOW locomotion testing.

This node subscribes to /joy and publishes /cmd_vel twist messages
specifically configured for PS5 controller layout.

PS5 Controller mapping:
- Left stick Y-axis: Linear X (forward/backward)
- Left stick X-axis: Linear Y (left/right strafe) 
- Right stick X-axis: Angular Z (rotation)
- L2/R2 triggers: Linear Z (up/down) - optional for jumping robots
- Cross button (X): Emergency stop
- Circle button: Reset/enable

Author: Generated for WLR Competition 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math


class PS5TwistController(Node):
    def __init__(self):
        super().__init__('ps5_twist_controller')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # PS5 controller button/axis mapping
        # Based on joy message: axes[2] and axes[5] are triggers (default 1.0)
        self.axis_linear_x = 1    # Left stick Y-axis (forward/backward)  
        self.axis_linear_y = 0    # Left stick X-axis (left/right strafe)
        self.axis_angular_z = 3   # Right stick X-axis (rotation) - was 2, but that's a trigger
        self.axis_linear_z = 4    # Right stick Y-axis (up/down) - optional
        
        # PS5 button indices (may vary by driver)
        self.button_emergency_stop = 0  # Cross button (X)
        self.button_enable = 1          # Circle button
        self.button_turbo = 4           # L1 shoulder button
        
        # Control parameters
        self.max_linear_speed = 1.0     # m/s
        self.max_angular_speed = 1.0    # rad/s
        self.max_strafe_speed = 0.5     # m/s (slower for safety)
        self.turbo_multiplier = 2.0     # Speed boost when turbo pressed
        
        # Deadzone to prevent drift
        self.deadzone = 0.1
        
        # Safety state
        self.enabled = True
        self.emergency_stop = False
        
        # Timer for regular publishing (even when no input)
        self.timer = self.create_timer(0.05, self.publish_twist)  # 20 Hz
        
        # Current twist command
        self.current_twist = Twist()
        
        self.get_logger().info("PS5 Twist Controller started!")
        self.get_logger().info("Controls:")
        self.get_logger().info("  Left stick Y: Forward/Backward")
        self.get_logger().info("  Left stick X: Strafe Left/Right") 
        self.get_logger().info("  Right stick X: Rotate Left/Right")
        self.get_logger().info("  Cross (X): Emergency Stop")
        self.get_logger().info("  Circle: Enable/Reset")
        self.get_logger().info("  L1: Turbo mode")
    
    def joy_callback(self, msg):
        """Process joystick input and update twist command."""
        # Check if we have enough axes for PS5 controller
        if len(msg.axes) < 6:  # PS5 typically has 6+ axes
            self.get_logger().warn(f"Insufficient joystick axes: got {len(msg.axes)}, need at least 6")
            return
            
        if len(msg.buttons) < 2:
            self.get_logger().warn(f"Insufficient joystick buttons: got {len(msg.buttons)}, need at least 2")
            return
        
        # Handle buttons first
        if len(msg.buttons) > self.button_emergency_stop and msg.buttons[self.button_emergency_stop]:
            if not self.emergency_stop:
                self.emergency_stop = True
                self.enabled = False
                self.get_logger().warn("EMERGENCY STOP activated!")
        
        if len(msg.buttons) > self.button_enable and msg.buttons[self.button_enable]:
            if self.emergency_stop:
                self.emergency_stop = False
                self.enabled = True
                self.get_logger().info("Controller enabled/reset")
        
        # If disabled, send zero twist
        if not self.enabled or self.emergency_stop:
            self.current_twist = Twist()
            return
        
        # Check for turbo mode
        turbo_active = (len(msg.buttons) > self.button_turbo and 
                       msg.buttons[self.button_turbo])
        speed_multiplier = self.turbo_multiplier if turbo_active else 1.0
        
        # Read axis values with deadzone
        linear_x_raw = self.apply_deadzone(msg.axes[self.axis_linear_x])
        linear_y_raw = self.apply_deadzone(msg.axes[self.axis_linear_y])
        angular_z_raw = self.apply_deadzone(msg.axes[self.axis_angular_z])
        
        # Debug logging for axis values (every 10th message to avoid spam)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        # if self.debug_counter % 50 == 0:  # Log every 50th message (2.5s at 20Hz)
        #     self.get_logger().info(
        #         f"Raw axes: [{', '.join([f'{ax:.3f}' for ax in msg.axes[:6]])}] "
        #         f"-> linear_x={linear_x_raw:.3f}, linear_y={linear_y_raw:.3f}, angular_z={angular_z_raw:.3f}"
        #     )
        
        # Create twist message
        twist = Twist()
        
        # Linear velocities (with speed scaling)
        twist.linear.x = linear_x_raw * self.max_linear_speed * speed_multiplier
        twist.linear.y = linear_y_raw * self.max_strafe_speed * speed_multiplier
        twist.linear.z = 0.0  # Most ground robots don't use this
        
        # Angular velocity
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z_raw * self.max_angular_speed * speed_multiplier
        
        self.current_twist = twist
        
        # Log significant commands for debugging
        if abs(twist.linear.x) > 0.1 or abs(twist.linear.y) > 0.1 or abs(twist.angular.z) > 0.1:
            self.get_logger().debug(
                f"Twist: linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}), "
                f"angular={twist.angular.z:.2f}"
                f"{' [TURBO]' if turbo_active else ''}"
            )
    
    def apply_deadzone(self, value):
        """Apply deadzone to prevent controller drift."""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale the remaining range to [0,1] or [-1,0]
        if value > 0:
            return (value - self.deadzone) / (1.0 - self.deadzone)
        else:
            return (value + self.deadzone) / (1.0 - self.deadzone)
    
    def publish_twist(self):
        """Publish the current twist command at regular intervals."""
        self.cmd_vel_pub.publish(self.current_twist)


def main(args=None):
    rclpy.init(args=args)
    
    node = PS5TwistController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PS5 twist controller...")
    finally:
        # Send stop command on shutdown
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
