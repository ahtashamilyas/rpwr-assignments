#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import actionlib
import tf2_ros
from tf2_ros import TransformException

class TortugabotController(Node):
    def __init__(self):
        super().__init__('tortugabot_controller')
        
        # Create subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_throttle',
            self.scan_callback,
            10
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.goal_collected_sub = self.create_subscription(
            String,
            '/goal_collected',
            self.goal_collected_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base/cmd_vel',
            10
        )
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize action client for MoveBase
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # Timer for regular processing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Control state
        self.mode = "manual"  # manual, wall_following, treasure_hunting
        self.scan_data = None
        self.joy_data = None
        self.current_cmd = Twist()
        
        # Treasure tracking
        self.treasures = []  # Will be populated with frame IDs
        self.trunk = []      # Currently collected treasures
        self.trash = []      # Trash items to avoid
        self.depot = []      # Drop-off locations
        
        # Find initial treasure and depot frames
        self.discover_frames()
        
        # Control parameters
        self.wall_follow_distance = 0.6
        self.collision_front_threshold = 0.5
        self.collision_side_threshold = 0.3
        
        # PS3 controller button mapping
        self.mode_toggle_button = 9  # PS button
        self.wall_follow_button = 3  # Square button
        self.treasure_hunt_button = 2  # Triangle button
        
        self.get_logger().info('Tortugabot Controller initialized')

    def discover_frames(self):
        """Discover treasure, trash and depot frames"""
        try:
            # Get all frames in TF tree
            all_frames = self.tf_buffer.all_frames_as_string()
            
            # Parse frame IDs
            for line in all_frames.split('\n'):
                if 'treasure' in line:
                    frame_id = line.strip()
                    if frame_id not in self.treasures:
                        self.treasures.append(frame_id)
                elif 'trash' in line:
                    frame_id = line.strip()
                    if frame_id not in self.trash:
                        self.trash.append(frame_id)
                elif 'depot' in line:
                    frame_id = line.strip()
                    if frame_id not in self.depot:
                        self.depot.append(frame_id)
            
            self.get_logger().info(f"Discovered {len(self.treasures)} treasures, {len(self.trash)} trash, {len(self.depot)} depots")
        
        except Exception as e:
            self.get_logger().error(f"Error discovering frames: {e}")

    def joy_callback(self, msg):
        """Process joystick input"""
        self.joy_data = msg
        
        # Process buttons for mode switching
        if len(msg.buttons) > self.mode_toggle_button:
            # Mode toggles
            if msg.buttons[self.mode_toggle_button] == 1 and not hasattr(self, 'last_mode_button'):
                # Cycle through modes
                if self.mode == "manual":
                    self.mode = "wall_following"
                elif self.mode == "wall_following":
                    self.mode = "treasure_hunting"
                else:
                    self.mode = "manual"
                    
                self.get_logger().info(f"Switched to {self.mode} mode")
                self.last_mode_button = True
                
            elif msg.buttons[self.mode_toggle_button] == 0:
                self.last_mode_button = False
        
        # In manual mode, process joystick for direct control
        if self.mode == "manual" and len(msg.axes) >= 2:
            cmd = Twist()
            cmd.linear.x = msg.axes[1] * 0.5  # Forward/back
            cmd.angular.z = msg.axes[2] * 1.0  # Rotation
            
            self.current_cmd = cmd

    def scan_callback(self, msg):
        """Store laser scan data"""
        self.scan_data = msg

    def goal_collected_callback(self, msg):
        """Handle goal collection events"""
        item = msg.data
        
        if item.startswith("treasure"):
            if item not in self.trunk and len(self.trunk) < 2:  # Max 2 treasures
                self.trunk.append(item)
                self.get_logger().info(f"Collected {item}. Trunk: {self.trunk}")
                
                # Remove from treasures list
                if item in self.treasures:
                    self.treasures.remove(item)
        
        elif item.startswith("trash"):
            if item not in self.trunk and len(self.trunk) < 2:
                self.trunk.append(item)
                self.get_logger().info(f"Collected {item}. Trunk: {self.trunk}")
                
                # Remove from trash list
                if item in self.trash:
                    self.trash.remove(item)
        
        elif item == "unloading":
            self.get_logger().info(f"Unloaded trunk: {self.trunk}")
            self.trunk.clear()

    def timer_callback(self):
        """Regular processing"""
        if self.scan_data is None:
            return
        
        # Process scan data for collision avoidance
        ranges = np.array(self.scan_data.ranges)
        valid_ranges = np.where(np.isfinite(ranges), ranges, np.inf)
        
        num_readings = len(valid_ranges)
        front_indices = np.arange(int(num_readings * 0.45), int(num_readings * 0.55))
        left_indices = np.arange(int(num_readings * 0.55), int(num_readings * 0.85))
        right_indices = np.arange(int(num_readings * 0.15), int(num_readings * 0.45))
        
        front_min = np.min(valid_ranges[front_indices])
        left_min = np.min(valid_ranges[left_indices])
        right_min = np.min(valid_ranges[right_indices])
        
        # Create command based on mode
        cmd = Twist()
        
        if self.mode == "manual":
            cmd = self.current_cmd
        elif self.mode == "wall_following":
            cmd = self.calculate_wall_following(valid_ranges, num_readings)
        elif self.mode == "treasure_hunting":
            # Only run treasure hunting logic every second to reduce computation
            if self.get_clock().now().nanoseconds % 1_000_000_000 < 100_000_000:
                self.run_treasure_hunting()
            return  # MoveBase will handle the command
        
        # Apply collision avoidance
        cmd = self.apply_collision_avoidance(cmd, front_min, left_min, right_min)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)

    def calculate_wall_following(self, ranges, num_readings):
        """Calculate command for wall following behavior"""
        cmd = Twist()
        
        # Focus on the right wall (90° to the right)
        right_wall_indices = np.arange(int(num_readings * 0.2), int(num_readings * 0.4))
        right_wall_distances = ranges[right_wall_indices]
        
        if np.min(right_wall_distances) < np.inf:
            # Wall detected
            right_wall_distance = np.min(right_wall_distances)
            
            # Calculate error from desired distance
            error = self.wall_follow_distance - right_wall_distance
            
            # P controller for angular velocity
            cmd.angular.z = 1.0 * error
            
            # Set forward velocity (slower when turning more)
            cmd.linear.x = 0.2 * (1.0 - min(1.0, abs(error)))
        else:
            # No wall detected, turn to find one
            cmd.angular.z = -0.3
            cmd.linear.x = 0.1
        
        return cmd

    def apply_collision_avoidance(self, cmd, front_dist, left_dist, right_dist):
        """Apply collision avoidance to a command"""
        # Check front collision
        if front_dist < self.collision_front_threshold:
            if cmd.linear.x > 0:
                scale = max(0.0, front_dist / self.collision_front_threshold)
                cmd.linear.x *= scale
                
                if front_dist < self.collision_front_threshold * 0.5:
                    cmd.linear.x = 0.0
        
        # Check side collisions
        if left_dist < self.collision_side_threshold and cmd.angular.z > 0:
            # Don't turn more to the left when left obstacle
            cmd.angular.z = min(0.0, cmd.angular.z)
        
        if right_dist < self.collision_side_threshold and cmd.angular.z < 0:
            # Don't turn more to the right when right obstacle
            cmd.angular.z = max(0.0, cmd.angular.z)
        
        return cmd

    def run_treasure_hunting(self):
        """Run the treasure hunting state machine"""
        # Priority:
        # 1. If trunk full (2 items) -> go to depot
        # 2. If treasures available -> go to nearest treasure
        # 3. Otherwise -> go to nearest trash
        
        if len(self.trunk) >= 2:
            # Trunk full, go to depot
            if self.depot:
                self.navigate_to_frame(self.depot[0])
                return
        
        # Look for treasures
        if self.treasures:
            # Find nearest treasure
            nearest_frame, _ = self.find_nearest_frame(self.treasures)
            if nearest_frame:
                self.navigate_to_frame(nearest_frame)
                return
        
        # If no treasures or depot with full trunk, go to trash
        if self.trash:
            nearest_frame, _ = self.find_nearest_frame(self.trash)
            if nearest_frame:
                self.navigate_to_frame(nearest_frame)
                return
    
    def find_nearest_frame(self, frame_list):
        """Find the nearest frame from a list"""
        nearest_frame = None
        min_distance = float('inf')
        
        for frame in frame_list:
            try:
                # Get transform from base to frame
                trans = self.tf_buffer.lookup_transform("base_footprint", frame, rclpy.time.Time())
                
                # Calculate distance
                distance = np.sqrt(
                    trans.transform.translation.x**2 + 
                    trans.transform.translation.y**2
                )
                
                if distance < min_distance:
                    min_distance = distance
                    nearest_frame = frame
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        
        return nearest_frame, min_distance

    def navigate_to_frame(self, frame_id):
        """Navigate to a specific TF frame"""
        try:
            # Get transform from map to frame
            trans = self.tf_buffer.lookup_transform("map", frame_id, rclpy.time.Time())
            
            # Create goal message
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set position
            goal.target_pose.pose.position.x = trans.transform.translation.x
            goal.target_pose.pose.position.y = trans.transform.translation.y
            goal.target_pose.pose.position.z = 0.0
            
            # Set orientation (identity quaternion - just face forward)
            goal.target_pose.pose.orientation.w = 1.0
            
            # Send goal
            self.get_logger().info(f"Navigating to {frame_id}")
            self.move_base_client.send_goal(goal)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Error navigating to {frame_id}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TortugabotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()