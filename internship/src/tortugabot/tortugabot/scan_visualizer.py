#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math

class ScanVisualizer(Node):
    def __init__(self):
        super().__init__('scan_visualizer')
        
        # Subscribers
        self.original_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.original_scan_callback,
            10
        )
        
        self.filtered_scan_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.filtered_scan_callback,
            10
        )
        
        # Publishers for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/scan_comparison_markers',
            10
        )
        
        # Store latest scans for comparison
        self.latest_original = None
        self.latest_filtered = None
        
        self.get_logger().info('Scan Visualizer Node initialized')

    def original_scan_callback(self, msg):
        """Store original scan for comparison"""
        self.latest_original = msg
        self.publish_comparison()

    def filtered_scan_callback(self, msg):
        """Store filtered scan for comparison"""
        self.latest_filtered = msg
        self.publish_comparison()

    def publish_comparison(self):
        """Publish visualization markers comparing original and filtered scans"""
        if self.latest_original is None or self.latest_filtered is None:
            return
        
        marker_array = MarkerArray()
        
        # Original scan points (red)
        original_marker = self.create_scan_marker(
            self.latest_original,
            'original_scan',
            [1.0, 0.0, 0.0, 0.5],  # Red, semi-transparent
            0.02
        )
        marker_array.markers.append(original_marker)
        
        # Filtered scan points (green)
        filtered_marker = self.create_scan_marker(
            self.latest_filtered,
            'filtered_scan',
            [0.0, 1.0, 0.0, 0.8],  # Green, more opaque
            0.03
        )
        marker_array.markers.append(filtered_marker)
        
        # Statistics text
        stats_marker = self.create_stats_marker()
        marker_array.markers.append(stats_marker)
        
        self.marker_pub.publish(marker_array)

    def create_scan_marker(self, scan_msg, marker_id, color, point_size):
        """Create a marker for visualizing laser scan points"""
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scan_comparison"
        marker.id = hash(marker_id) % 1000  # Simple hash for unique ID
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = point_size
        marker.scale.y = point_size
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Convert polar to cartesian coordinates
        angle = scan_msg.angle_min
        for range_val in scan_msg.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)):
                if scan_msg.range_min <= range_val <= scan_msg.range_max:
                    point = Point()
                    point.x = range_val * math.cos(angle)
                    point.y = range_val * math.sin(angle)
                    point.z = 0.0
                    marker.points.append(point)
            angle += scan_msg.angle_increment
        
        return marker

    def create_stats_marker(self):
        """Create a text marker showing statistics"""
        marker = Marker()
        marker.header.frame_id = self.latest_original.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scan_comparison"
        marker.id = 999
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Calculate statistics
        original_valid = sum(1 for r in self.latest_original.ranges 
                           if not (math.isnan(r) or math.isinf(r)))
        filtered_valid = sum(1 for r in self.latest_filtered.ranges 
                           if not (math.isnan(r) or math.isinf(r)))
        
        reduction_percent = (1 - len(self.latest_filtered.ranges) / len(self.latest_original.ranges)) * 100
        
        marker.text = (
            f"Original: {len(self.latest_original.ranges)} points ({original_valid} valid)\n"
            f"Filtered: {len(self.latest_filtered.ranges)} points ({filtered_valid} valid)\n"
            f"Reduction: {reduction_percent:.1f}%\n"
            f"Red: Original, Green: Filtered"
        )
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    
    visualizer = ScanVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()