#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollowing(Node):
    def __init__(self):
        super().__init__('wall_following')
        
        # Publisher for robot movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        
        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Wall following parameters
        self.desired_distance = 0.5  # Desired distance from wall (m)
        self.linear_speed = 0.2  # Forward speed (m/s)
        self.max_angular_speed = 1.0  # Maximum turning speed (rad/s)
        
        # PID controller parameters for distance control
        self.kp = 2.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.5  # Derivative gain
        
        # PID state variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        
        # Safety parameters
        self.front_obstacle_distance = 0.4  # Distance to detect front obstacles
        self.min_wall_distance = 0.2  # Minimum distance to wall
        self.max_wall_distance = 1.5  # Maximum distance to consider wall following
        
        self.get_logger().info('Wall Following Node Started')

    def scan_callback(self, msg):
        """Process laser scan data and implement wall following behavior"""
        
        # Get distances to the right side and front
        right_distance = self.get_right_distance(msg)
        front_distance = self.get_front_distance(msg)
        
        # Create movement command
        twist = Twist()
        
        # Check for front obstacle
        if front_distance < self.front_obstacle_distance:
            # Turn left when obstacle ahead
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular_speed
            self.get_logger().info(f'Front obstacle detected at {front_distance:.2f}m, turning left')
        elif right_distance > self.max_wall_distance:
            # No wall detected on right, turn right to find wall
            twist.linear.x = self.linear_speed * 0.5
            twist.angular.z = -self.max_angular_speed * 0.5
            self.get_logger().info('No wall detected, searching for wall')
        elif right_distance < self.min_wall_distance:
            # Too close to wall, turn left
            twist.linear.x = self.linear_speed * 0.5
            twist.angular.z = self.max_angular_speed * 0.8
            self.get_logger().info(f'Too close to wall ({right_distance:.2f}m), turning left')
        else:
            # Normal wall following using PID control
            error = self.desired_distance - right_distance
            angular_velocity = self.calculate_pid_control(error)
            
            twist.linear.x = self.linear_speed
            twist.angular.z = angular_velocity
            
            self.get_logger().info(f'Following wall: distance={right_distance:.2f}m, error={error:.2f}m')
        
        # Publish the movement command
        self.cmd_vel_pub.publish(twist)

    def get_right_distance(self, scan_msg):
        """Get distance to the right side of the robot"""
        total_points = len(scan_msg.ranges)
        
        # Calculate index for 90 degrees to the right (-π/2 radians)
        # In most laser scans, -π/2 is at 3/4 of the array
        right_index = int(3 * total_points / 4)
        
        # Take average of several points around the right side for stability
        sample_range = 10
        start_idx = max(0, right_index - sample_range // 2)
        end_idx = min(total_points, right_index + sample_range // 2)
        
        valid_distances = []
        for i in range(start_idx, end_idx):
            distance = scan_msg.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance) and distance > 0:
                valid_distances.append(distance)
        
        if valid_distances:
            return sum(valid_distances) / len(valid_distances)
        else:
            return float('inf')

    def get_front_distance(self, scan_msg):
        """Get minimum distance in front of the robot"""
        total_points = len(scan_msg.ranges)
        
        # Check front sector (center of the scan)
        center_index = total_points // 2
        sample_range = 20  # Check wider range for obstacles
        
        start_idx = max(0, center_index - sample_range // 2)
        end_idx = min(total_points, center_index + sample_range // 2)
        
        min_distance = float('inf')
        for i in range(start_idx, end_idx):
            distance = scan_msg.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance) and distance > 0:
                min_distance = min(min_distance, distance)
        
        # Also check the beginning and end of array (which might represent front)
        for i in range(min(sample_range // 2, total_points)):
            distance = scan_msg.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance) and distance > 0:
                min_distance = min(min_distance, distance)
        
        for i in range(max(0, total_points - sample_range // 2), total_points):
            distance = scan_msg.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance) and distance > 0:
                min_distance = min(min_distance, distance)
        
        return min_distance

    def calculate_pid_control(self, error):
        """Calculate PID control output for angular velocity"""
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral_error += error
        integral = self.ki * self.integral_error
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error)
        self.previous_error = error
        
        # Calculate total control output
        control_output = proportional + integral + derivative
        
        # Limit the angular velocity
        angular_velocity = max(-self.max_angular_speed, 
                             min(self.max_angular_speed, control_output))
        
        return angular_velocity

def main(args=None):
    rclpy.init(args=args)
    
    wall_following = WallFollowing()
    
    try:
        rclpy.spin(wall_following)
    except KeyboardInterrupt:
        wall_following.get_logger().info('Shutting down wall following node.')
    
    # Stop the robot before shutting down
    twist = Twist()
    wall_following.cmd_vel_pub.publish(twist)
    
    wall_following.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()