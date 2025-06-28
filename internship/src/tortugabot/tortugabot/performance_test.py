#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
import numpy as np
from collections import deque

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Subscribers for both original and filtered scans
        self.original_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.original_callback,
            10
        )
        
        self.filtered_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.filtered_callback,
            10
        )
        
        # Performance tracking
        self.original_times = deque(maxlen=100)
        self.filtered_times = deque(maxlen=100)
        self.original_count = 0
        self.filtered_count = 0
        
        # Timer for periodic reporting
        self.timer = self.create_timer(5.0, self.report_performance)
        
        self.get_logger().info('Performance Monitor started')

    def original_callback(self, msg):
        """Track original scan frequency"""
        current_time = time.time()
        self.original_times.append(current_time)
        self.original_count += 1

    def filtered_callback(self, msg):
        """Track filtered scan frequency"""
        current_time = time.time()
        self.filtered_times.append(current_time)
        self.filtered_count += 1

    def calculate_frequency(self, timestamps):
        """Calculate frequency from timestamps"""
        if len(timestamps) < 2:
            return 0.0
        
        # Calculate intervals between consecutive timestamps
        intervals = []
        for i in range(1, len(timestamps)):
            intervals.append(timestamps[i] - timestamps[i-1])
        
        if not intervals:
            return 0.0
        
        avg_interval = np.mean(intervals)
        if avg_interval <= 0:
            return 0.0
        
        return 1.0 / avg_interval

    def report_performance(self):
        """Report performance statistics"""
        original_freq = self.calculate_frequency(list(self.original_times))
        filtered_freq = self.calculate_frequency(list(self.filtered_times))
        
        self.get_logger().info('=== PERFORMANCE REPORT ===')
        self.get_logger().info(f'Original scan frequency: {original_freq:.1f} Hz (target: 40 Hz)')
        self.get_logger().info(f'Filtered scan frequency: {filtered_freq:.1f} Hz')
        self.get_logger().info(f'Total messages - Original: {self.original_count}, Filtered: {self.filtered_count}')
        
        # Check if we're meeting the 40Hz requirement
        if original_freq > 35.0:  # Allow some tolerance
            self.get_logger().info('✓ Meeting 40Hz requirement for original scan')
        else:
            self.get_logger().warn('⚠ Original scan frequency below 40Hz target')
        
        if filtered_freq > 35.0:
            self.get_logger().info('✓ Filter keeping up with 40Hz requirement')
        else:
            self.get_logger().warn('⚠ Filtered scan frequency below target - filter may be too slow')
        
        self.get_logger().info('========================')


def main(args=None):
    rclpy.init(args=args)
    
    monitor = PerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()