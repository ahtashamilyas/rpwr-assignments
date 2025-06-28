#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from collections import deque

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        
        # Parameters
        self.declare_parameter('min_range', 0.1)  # Remove points closer than this
        self.declare_parameter('max_range', 10.0)  # Remove points farther than this
        self.declare_parameter('angular_resolution', 0.02)  # Target angular resolution (radians)
        self.declare_parameter('median_filter_size', 3)  # Size of median filter window
        self.declare_parameter('pole_reflection_threshold', 0.05)  # Distance threshold to remove pole reflections
        
        # Get parameters
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.angular_resolution = self.get_parameter('angular_resolution').value
        self.median_filter_size = self.get_parameter('median_filter_size').value
        self.pole_threshold = self.get_parameter('pole_reflection_threshold').value
        
        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10
        )
        
        # For visualization comparison
        self.original_scan_pub = self.create_publisher(
            LaserScan,
            '/scan_original',
            10
        )
        
        # Performance monitoring
        self.processing_times = deque(maxlen=100)
        self.frame_count = 0
        
        self.get_logger().info('Scan Filter Node initialized')
        self.get_logger().info(f'Parameters: min_range={self.min_range}, max_range={self.max_range}')
        self.get_logger().info(f'Angular resolution: {self.angular_resolution} rad ({math.degrees(self.angular_resolution):.1f} deg)')

    def scan_callback(self, msg):
        """Main callback for processing laser scan messages"""
        start_time = self.get_clock().now()
        
        # Create filtered scan message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = msg.header.frame_id
        
        # Step 1: Basic range filtering and noise removal
        filtered_ranges = self.range_filter(msg.ranges)
        
        # Step 2: Remove pole reflections (very close consecutive points)
        filtered_ranges = self.remove_pole_reflections(filtered_ranges)
        
        # Step 3: Apply median filter to reduce noise
        filtered_ranges = self.median_filter(filtered_ranges)
        
        # Step 4: Reduce resolution by downsampling
        filtered_ranges, new_angle_increment = self.downsample_scan(
            filtered_ranges, msg.angle_increment
        )
        
        # Update scan parameters
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_min + (len(filtered_ranges) - 1) * new_angle_increment
        filtered_msg.angle_increment = new_angle_increment
        filtered_msg.time_increment = msg.time_increment * (new_angle_increment / msg.angle_increment)
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = max(msg.range_min, self.min_range)
        filtered_msg.range_max = min(msg.range_max, self.max_range)
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = []  # Clear intensities to save bandwidth
        
        # Publish filtered scan
        self.filtered_scan_pub.publish(filtered_msg)
        
        # Publish original for comparison (with same frame_id for visualization)
        original_msg = LaserScan()
        original_msg.header = msg.header
        original_msg.header.frame_id = msg.header.frame_id + "_original"
        original_msg.angle_min = msg.angle_min
        original_msg.angle_max = msg.angle_max
        original_msg.angle_increment = msg.angle_increment
        original_msg.time_increment = msg.time_increment
        original_msg.scan_time = msg.scan_time
        original_msg.range_min = msg.range_min
        original_msg.range_max = msg.range_max
        original_msg.ranges = list(msg.ranges)
        original_msg.intensities = []
        self.original_scan_pub.publish(original_msg)
        
        # Performance monitoring
        end_time = self.get_clock().now()
        processing_time = (end_time - start_time).nanoseconds / 1e6  # Convert to milliseconds
        self.processing_times.append(processing_time)
        
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            avg_time = np.mean(self.processing_times)
            self.get_logger().info(
                f'Processed {self.frame_count} frames. '
                f'Avg processing time: {avg_time:.2f}ms. '
                f'Points: {len(msg.ranges)} -> {len(filtered_ranges)} '
                f'({len(filtered_ranges)/len(msg.ranges)*100:.1f}%)'
            )

    def range_filter(self, ranges):
        """Remove points that are too close or too far"""
        filtered = []
        for range_val in ranges:
            if math.isnan(range_val) or math.isinf(range_val):
                filtered.append(float('inf'))
            elif range_val < self.min_range or range_val > self.max_range:
                filtered.append(float('inf'))
            else:
                filtered.append(range_val)
        return filtered

    def remove_pole_reflections(self, ranges):
        """Remove pole reflections by detecting isolated close points"""
        if len(ranges) < 3:
            return ranges
        
        filtered = list(ranges)
        n = len(ranges)
        
        for i in range(1, n - 1):
            current = ranges[i]
            prev_val = ranges[i - 1]
            next_val = ranges[i + 1]
            
            # Skip if current point is invalid
            if math.isinf(current) or math.isnan(current):
                continue
                
            # Check if current point is much closer than neighbors
            prev_valid = not (math.isinf(prev_val) or math.isnan(prev_val))
            next_valid = not (math.isinf(next_val) or math.isnan(next_val))
            
            if prev_valid and next_valid:
                # If current point is much closer than both neighbors, likely a reflection
                if (current < prev_val - self.pole_threshold and 
                    current < next_val - self.pole_threshold):
                    filtered[i] = float('inf')
            elif prev_valid:
                if current < prev_val - self.pole_threshold:
                    filtered[i] = float('inf')
            elif next_valid:
                if current < next_val - self.pole_threshold:
                    filtered[i] = float('inf')
        
        return filtered

    def median_filter(self, ranges):
        """Apply median filter to reduce noise"""
        if self.median_filter_size <= 1:
            return ranges
        
        filtered = list(ranges)
        n = len(ranges)
        half_window = self.median_filter_size // 2
        
        for i in range(half_window, n - half_window):
            window = []
            for j in range(i - half_window, i + half_window + 1):
                val = ranges[j]
                if not (math.isinf(val) or math.isnan(val)):
                    window.append(val)
            
            if len(window) >= self.median_filter_size // 2:
                filtered[i] = np.median(window)
        
        return filtered

    def downsample_scan(self, ranges, original_increment):
        """Reduce resolution by downsampling similar adjacent points"""
        if len(ranges) <= 1:
            return ranges, original_increment
        
        # Calculate downsampling factor
        downsample_factor = max(1, int(self.angular_resolution / original_increment))
        
        if downsample_factor <= 1:
            return ranges, original_increment
        
        # Downsample by taking every nth point, using median of nearby points
        downsampled = []
        new_increment = original_increment * downsample_factor
        
        for i in range(0, len(ranges), downsample_factor):
            # Take median of points in the downsampling window
            window_end = min(i + downsample_factor, len(ranges))
            window_values = []
            
            for j in range(i, window_end):
                val = ranges[j]
                if not (math.isinf(val) or math.isnan(val)):
                    window_values.append(val)
            
            if window_values:
                downsampled.append(np.median(window_values))
            else:
                downsampled.append(float('inf'))
        
        return downsampled, new_increment


def main(args=None):
    rclpy.init(args=args)
    
    scan_filter = ScanFilter()
    
    try:
        rclpy.spin(scan_filter)
    except KeyboardInterrupt:
        pass
    finally:
        scan_filter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()