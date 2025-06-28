# Laser Scan Filter for Tortugabot

This implementation provides a real-time laser scan filter designed to run at 40Hz, reducing noise and computational load while maintaining essential obstacle detection capabilities.

## Features

- **Range Filtering**: Removes points that are too close (pole reflections) or too far away
- **Pole Reflection Removal**: Detects and removes spurious reflections from nearby poles/objects
- **Median Filtering**: Reduces noise while preserving edges
- **Resolution Reduction**: Downsamples scan data to reduce computational load
- **Real-time Performance**: Optimized for 40Hz operation
- **Visualization**: Compare original and filtered scans in RViz

## Files

- `scan_filter.py` - Main filter node
- `scan_filter_launch.py` - Launch file with configurable parameters
- `scan_visualizer.py` - Visualization node for comparing scans
- `performance_test.py` - Performance monitoring script

## Installation

1. Copy the Python files to your ROS2 workspace:
```bash
# Assuming you're in your tortugabot workspace
cp scan_filter.py src/tortugabot/tortugabot/
cp scan_filter_launch.py src/tortugabot/launch/
cp scan_visualizer.py src/tortugabot/tortugabot/
cp performance_test.py src/tortugabot/tortugabot/
```

2. Make the scripts executable:
```bash
chmod +x src/tortugabot/tortugabot/scan_filter.py
chmod +x src/tortugabot/tortugabot/scan_visualizer.py
chmod +x src/tortugabot/tortugabot/performance_test.py
```

3. Update your package's `setup.py` to include the new executables:
```python
entry_points={
    'console_scripts': [
        'scan_filter = tortugabot.scan_filter:main',
        'scan_visualizer = tortugabot.scan_visualizer:main',
        'performance_test = tortugabot.performance_test:main',
    ],
},
```

4. Build the package:
```bash
colcon build --packages-select tortugabot
source install/setup.bash
```

## Usage

### Basic Usage

Start the scan filter with default parameters:
```bash
ros2 run tortugabot scan_filter
```

Or use the launch file:
```bash
ros2 launch tortugabot scan_filter_launch.py
```

### With Custom Parameters

```bash
ros2 launch tortugabot scan_filter_launch.py \
    min_range:=0.15 \
    max_range:=8.0 \
    angular_resolution:=0.03 \
    median_filter_size:=5
```

### Visualization

To compare original and filtered scans in RViz:

1. Start the filter:
```bash
ros2 launch tortugabot scan_filter_launch.py
```

2. Start the visualizer:
```bash
ros2 run tortugabot scan_visualizer
```

3. Open RViz and add:
   - LaserScan display for `/scan` (original)
   - LaserScan display for `/scan_filtered` (filtered)
   - MarkerArray display for `/scan_comparison_markers` (comparison)

### Performance Monitoring

Monitor the filter's performance:
```bash
ros2 run tortugabot performance_test
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_range` | 0.1 | Minimum range to keep (meters) |
| `max_range` | 10.0 | Maximum range to keep (meters) |
| `angular_resolution` | 0.02 | Target angular resolution (radians, ~1.15°) |
| `median_filter_size` | 3 | Size of median filter window |
| `pole_reflection_threshold` | 0.05 | Distance threshold for pole reflection removal |

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan) - Original laser scan data

### Published Topics
- `/scan_filtered` (sensor_msgs/LaserScan) - Filtered laser scan data
- `/scan_original` (sensor_msgs/LaserScan) - Copy of original scan for visualization
- `/scan_comparison_markers` (visualization_msgs/MarkerArray) - Visualization markers

## Filter Algorithm

The filter applies the following steps in sequence:

1. **Range Filtering**: Remove points outside min/max range
2. **Pole Reflection Removal**: Detect isolated close points that are likely reflections
3. **Median Filtering**: Apply median filter to reduce noise
4. **Downsampling**: Reduce angular resolution by combining nearby points

## Performance Optimization

The filter is optimized for real-time operation:

- Uses NumPy for efficient array operations
- Minimal memory allocation in the main loop
- Vectorized operations where possible
- Configurable parameters to balance quality vs. speed

## Testing with Bagfiles

Record a bagfile for offline testing:
```bash
ros2 bag record /scan -o test_scan
```

Play it back in a loop:
```bash
ros2 bag play test_scan -l
```

Run the filter and monitor performance:
```bash
# Terminal 1
ros2 run tortugabot scan_filter

# Terminal 2
ros2 run tortugabot performance_test
```

## Integration with Other Nodes

The filtered scan can be used as input to other navigation components:

```bash
# Example: Use filtered scan for navigation
ros2 launch nav2_bringup navigation_launch.py
# Make sure to remap the scan topic to /scan_filtered
```

## Troubleshooting

### Filter is too slow
- Increase `angular_resolution` to reduce points
- Decrease `median_filter_size`
- Increase `min_range` to filter out more close points

### Too much noise in output
- Decrease `angular_resolution` for higher resolution
- Increase `median_filter_size`
- Adjust `pole_reflection_threshold`

### Missing important obstacles
- Decrease `min_range` and `max_range` as appropriate
- Decrease `angular_resolution` for higher resolution
- Check that obstacles are within the configured range limits

## Expected Performance

- **Processing Time**: ~1-3ms per scan (well within 25ms budget for 40Hz)
- **Data Reduction**: 30-70% fewer points depending on configuration
- **Latency**: Minimal additional latency (~1ms)

This filter should easily handle the 40Hz requirement while significantly reducing noise and computational load for downstream processing.