#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        
        # Publisher for robot movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        
        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Robot control parameters
        self.linear_speed = 0.3  # Forward speed (m/s)
        self.angular_speed = 0.5  # Turning speed (rad/s)
        self.collision_distance = 0.8  # Distance threshold for collision detection (m)
        self.front_angle_range = 30  # Angle range to check in front (degrees)
        
        # State variables
        self.turning = False
        self.turn_direction = 1  # 1 for left, -1 for right
        
        self.get_logger().info('Collision Avoidance Node Started')

    def scan_callback(self, msg):
        """Process laser scan data and control robot movement"""
        
        # Check for obstacles in front of the robot
        obstacle_detected = self.check_front_obstacle(msg)
        
        # Create movement command
        twist = Twist()
        
        if obstacle_detected and not self.turning:
            # Start turning when obstacle is detected
            self.turning = True
            # Randomly choose turn direction (or you can implement logic to choose best direction)
            self.turn_direction = 1  # Turn left by default
            self.get_logger().info('Obstacle detected! Starting to turn.')
        
        if self.turning:
            # Keep turning until path is clear
            if not obstacle_detected:
                self.turning = False
                self.get_logger().info('Path clear! Moving forward.')
            else:
                # Continue turning
                twist.angular.z = self.turn_direction * self.angular_speed
        else:
            # Move forward when path is clear
            twist.linear.x = self.linear_speed
        
        # Publish the movement command
        self.cmd_vel_pub.publish(twist)

    def check_front_obstacle(self, scan_msg):
        """Check if there's an obstacle in front of the robot"""
        
        # Convert angle range to radians
        angle_range_rad = math.radians(self.front_angle_range)
        
        # Calculate indices for the front sector
        # LaserScan typically goes from -π to π, with 0 being straight ahead
        total_points = len(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        
        # Calculate how many points correspond to our front angle range
        points_in_range = int(angle_range_rad / angle_increment)
        
        # Check front-center region (around index 0, which is straight ahead)
        # Since the scan might be ordered differently, we check both sides of center
        center_index = total_points // 2
        start_index = center_index - points_in_range // 2
        end_index = center_index + points_in_range // 2
        
        # Also check the very front (index 0 area) and the end (which might also be front)
        front_indices = []
        
        # Add center region
        for i in range(max(0, start_index), min(total_points, end_index)):
            front_indices.append(i)
        
        # Add front region (beginning and end of array, as they represent front angles)
        for i in range(min(points_in_range // 2, total_points)):
            front_indices.append(i)
        for i in range(max(0, total_points - points_in_range // 2), total_points):
            front_indices.append(i)
        
        # Check if any point in the front region is too close
        for i in front_indices:
            if i < len(scan_msg.ranges):
                distance = scan_msg.ranges[i]
                # Check if distance is valid and within collision threshold
                if not math.isinf(distance) and not math.isnan(distance):
                    if distance < self.collision_distance:
                        return True
        
        return False

def main(args=None):
    rclpy.init(args=args)
    
    collision_avoidance = CollisionAvoidance()
    
    try:
        rclpy.spin(collision_avoidance)
    except KeyboardInterrupt:
        collision_avoidance.get_logger().info('Shutting down collision avoidance node.')
    
    # Stop the robot before shutting down
    twist = Twist()
    collision_avoidance.cmd_vel_pub.publish(twist)
    
    collision_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()