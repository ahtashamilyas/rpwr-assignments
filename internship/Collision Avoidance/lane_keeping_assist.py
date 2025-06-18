#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
import numpy as np

class LaneKeepingAssist(Node):
    def __init__(self):
        super().__init__('lane_keeping_assist')
        
        # Create subscription to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_throttle',
            self.scan_callback,
            10
        )
        
        # Create subscription to joystick inputs
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10  
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base/cmd_vel',
            10
        )
        
        # Create a timer for regular updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Store the latest velocity command
        self.current_cmd = Twist()
        
        # Store joystick state
        self.joy_data = None
        
        # Store laser scan data
        self.scan_data = None
        
        # Control parameters
        self.autonomous_mode = False  # Start in manual mode
        self.mode_button = 9  # PS button on PS3 controller (adjust as needed)
        
        # Lane keeping parameters
        self.target_wall_distance = 0.6  # Desired distance from wall (in meters)
        self.wall_follow_p_gain = 1.0  # Proportional gain for wall following
        
        # Safety parameters
        self.min_front_distance = 0.5
        self.min_side_distance = 0.3
        
        self.get_logger().info('Lane Keeping Assist node initialized')

    def joy_callback(self, msg):
        """Process joystick input"""
        self.joy_data = msg
        
        # Check for mode toggle button
        if len(msg.buttons) > self.mode_button and msg.buttons[self.mode_button] == 1:
            # Toggle mode only on button press (not hold)
            if not hasattr(self, 'last_mode_button_state') or self.last_mode_button_state == 0:
                self.autonomous_mode = not self.autonomous_mode
                mode_str = "autonomous" if self.autonomous_mode else "manual"
                self.get_logger().info(f'Switched to {mode_str} mode')
            
            self.last_mode_button_state = 1
        else:
            self.last_mode_button_state = 0
        
        # Process joystick axes for manual control
        if not self.autonomous_mode and len(msg.axes) >= 2:
            # Create Twist message from joystick
            # (assuming axes[1] is forward/back and axes[0] or axes[2] is rotation)
            cmd = Twist()
            cmd.linear.x = msg.axes[1] * 0.5  # Scale down for safety
            cmd.angular.z = msg.axes[2] * 1.0  # Use right stick for rotation
            
            # Store raw command (before collision avoidance)
            self.current_cmd = cmd

    def scan_callback(self, msg):
        """Store laser scan data"""
        self.scan_data = msg

    def timer_callback(self):
        """Process sensor data and generate commands"""
        if self.scan_data is None:
            return
        
        # Process the scan data to find distances
        ranges = np.array(self.scan_data.ranges)
        
        # Filter out invalid readings
        valid_ranges = np.where(np.isfinite(ranges), ranges, np.inf)
        
        # Define angular regions
        num_readings = len(valid_ranges)
        
        front_indices = np.arange(int(num_readings * 0.45), int(num_readings * 0.55))
        left_indices = np.arange(int(num_readings * 0.55), int(num_readings * 0.85))
        right_indices = np.arange(int(num_readings * 0.15), int(num_readings * 0.45))
        
        # Get minimum distances in each region
        front_min = np.min(valid_ranges[front_indices])
        left_min = np.min(valid_ranges[left_indices])
        right_min = np.min(valid_ranges[right_indices])
        
        # Create command based on mode
        cmd = Twist()
        
        if self.autonomous_mode:
            # Autonomous mode: wall following
            cmd = self.calculate_wall_following()
        else:
            # Manual mode: use stored joystick command
            cmd = Twist()
            cmd.linear.x = self.current_cmd.linear.x
            cmd.angular.z = self.current_cmd.angular.z
        
        # Apply collision avoidance in both modes
        cmd = self.apply_collision_avoidance(cmd, front_min, left_min, right_min)
        
        # Publish the command
        self.cmd_vel_pub.publish(cmd)
    
    def calculate_wall_following(self):
        """Implement wall following behavior"""
        cmd = Twist()
        
        if self.scan_data is None:
            return cmd
        
        # Get ranges
        ranges = np.array(self.scan_data.ranges)
        valid_ranges = np.where(np.isfinite(ranges), ranges, np.inf)
        
        # Find the right wall (90° to the right)
        num_readings = len(valid_ranges)
        right_wall_indices = np.arange(int(num_readings * 0.2), int(num_readings * 0.4))
        right_wall_distances = valid_ranges[right_wall_indices]
        
        if np.min(right_wall_distances) < np.inf:
            # Wall is detected
            right_wall_distance = np.min(right_wall_distances)
            
            # Calculate error
            error = self.target_wall_distance - right_wall_distance
            
            # P controller for angular velocity
            cmd.angular.z = self.wall_follow_p_gain * error
            
            # Set forward velocity
            cmd.linear.x = 0.2  # Slow and steady
        else:
            # No wall detected, rotate to find one
            cmd.angular.z = -0.3  # Rotate right to find a wall
            cmd.linear.x = 0.1   # Slow forward
        
        return cmd
    
    def apply_collision_avoidance(self, cmd, front_dist, left_dist, right_dist):
        """Apply collision avoidance to a command"""
        # Check front collision
        if front_dist < self.min_front_distance:
            # Stop forward motion if obstacle is close
            if cmd.linear.x > 0:
                scale_factor = max(0.0, front_dist / self.min_front_distance)
                cmd.linear.x *= scale_factor
                
                # If very close, stop completely
                if front_dist < self.min_front_distance * 0.5:
                    cmd.linear.x = 0.0
        
        # Check side collisions (adjust turning if needed)
        if left_dist < self.min_side_distance and cmd.angular.z > 0:
            # Reduce turning to the left when obstacle on left
            cmd.angular.z = 0.0
        
        if right_dist < self.min_side_distance and cmd.angular.z < 0:
            # Reduce turning to the right when obstacle on right
            cmd.angular.z = 0.0
        
        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = LaneKeepingAssist()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()