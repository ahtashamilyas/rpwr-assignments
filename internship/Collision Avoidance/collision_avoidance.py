#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        
        # Create subscription to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_throttle',  # Use throttled scan to reduce processing load
            self.scan_callback,
            10
        )
        
        # Publisher to modify velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base/cmd_vel',
            10
        )
        
        # Create subscription to original velocity commands (from controller)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/joy_teleop/cmd_vel',  # Assuming this is from the PS3 controller
            self.cmd_vel_callback,
            10
        )
        
        # Store the latest velocity command
        self.current_cmd = Twist()
        
        # Define safety thresholds (in meters)
        self.front_threshold = 0.5  # Distance to start avoiding in front
        self.side_threshold = 0.3   # Distance to start avoiding on sides
        
        self.get_logger().info('Collision avoidance node initialized')

    def scan_callback(self, msg):
        """Process laser scan data to detect potential collisions"""
        # Define regions (indices will depend on your LiDAR's configuration)
        # For a typical laser with 0 degrees at the front and going counterclockwise:
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings
        ranges = np.where(np.isnan(ranges), np.inf, ranges)
        ranges = np.where(ranges < msg.range_min, np.inf, ranges)
        ranges = np.where(ranges > msg.range_max, np.inf, ranges)
        
        # Define angular regions
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # Calculate number of readings in total scan
        num_readings = len(ranges)
        
        # Define regions (adjust indices based on your LiDAR orientation)
        front_indices = np.arange(int(num_readings * 0.45), int(num_readings * 0.55))
        left_indices = np.arange(int(num_readings * 0.55), int(num_readings * 0.85))
        right_indices = np.arange(int(num_readings * 0.15), int(num_readings * 0.45))
        
        # Get minimum distances in each region
        front_min = np.min(ranges[front_indices])
        left_min = np.min(ranges[left_indices])
        right_min = np.min(ranges[right_indices])
        
        self.get_logger().debug(f'Min distances - Front: {front_min:.2f}, Left: {left_min:.2f}, Right: {right_min:.2f}')
        
        # Check for collisions and modify velocity
        self.avoid_collision(front_min, left_min, right_min)

    def cmd_vel_callback(self, msg):
        """Store the latest velocity command from the controller"""
        self.current_cmd = msg
    
    def avoid_collision(self, front_dist, left_dist, right_dist):
        """Modify velocity commands based on obstacle distances"""
        cmd = Twist()
        cmd.linear.x = self.current_cmd.linear.x
        cmd.angular.z = self.current_cmd.angular.z
        
        # Flag to track if we've modified the command
        modified = False
        
        # Check front collision
        if front_dist < self.front_threshold:
            # Slow down proportionally to how close we are
            scale_factor = max(0.0, front_dist / self.front_threshold)
            cmd.linear.x = self.current_cmd.linear.x * scale_factor
            
            # If very close or moving forward, enforce stop
            if front_dist < self.front_threshold * 0.5 and cmd.linear.x > 0:
                cmd.linear.x = 0.0
            
            modified = True
            self.get_logger().info(f'Front obstacle detected at {front_dist:.2f}m, adjusting speed')
        
        # Check side collisions and adjust turning
        if left_dist < self.side_threshold:
            # Avoid turning more to the left
            if cmd.angular.z > 0:
                cmd.angular.z = 0.0
            # Turn right to move away
            if left_dist < self.side_threshold * 0.7:
                cmd.angular.z = -0.3  # Turn right
            
            modified = True
            self.get_logger().info(f'Left obstacle detected at {left_dist:.2f}m, adjusting direction')
        
        if right_dist < self.side_threshold:
            # Avoid turning more to the right
            if cmd.angular.z < 0:
                cmd.angular.z = 0.0
            # Turn left to move away
            if right_dist < self.side_threshold * 0.7:
                cmd.angular.z = 0.3  # Turn left
            
            modified = True
            self.get_logger().info(f'Right obstacle detected at {right_dist:.2f}m, adjusting direction')
        
        # If we modified the command, publish it
        if modified:
            self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()