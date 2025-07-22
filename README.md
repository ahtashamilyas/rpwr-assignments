# RPWR Assignments - Robot Programming with ROS

This repository contains assignments and projects from the Robot Programming with ROS course, including advanced robotics implementations developed during my internship. The projects demonstrate real-world applications of ROS2, navigation, sensor processing, and autonomous robot behavior.

## 🎯 Internship Projects Overview

During my internship, I successfully implemented three major robotics challenges that showcase advanced ROS2 capabilities and real-time sensor processing:

### 1. 🚗 Lane Keep Assistant Robot
An intelligent collision avoidance system that enables autonomous navigation with dynamic obstacle detection and response.

**Key Features:**
- **Collision Detection Windows**: Uses `/scan` data to create detection zones around the robot
- **Adaptive Behavior System**:
  - **Front Obstacles**: Robot stops and rotates to find clear path
  - **Left Obstacles**: Turns right while maintaining forward momentum
  - **Right Obstacles**: Turns left while maintaining forward momentum
- **Real-time Visualization**: Publishes markers to visualize obstacle positions in map frame
- **Persistent Obstacle Tracking**: Obstacles remain visible for several seconds to aid in navigation planning

**Demo Video:**
![Lane Keep Assistant](Videos/LanekeepingAssistant.mp4)

---

### 2. 📡 Real-Time Laser Scan Filtering For Robotics
A high-performance laser scan processing system designed for 40Hz real-time operation, optimizing computational efficiency while maintaining critical obstacle detection accuracy.

**Technical Implementation:**
- **🔍 Initial Range Filtering**: Removes invalid and out-of-range measurements
- **📊 Median Filtering**: Eliminates noise spikes and random measurement errors
- **⚡ Resolution Reduction (Downsampling)**: Reduces computational load while preserving essential data
- **🎯 Real-Time Performance**: Maintains 40Hz operation for responsive robot control
- **🚫 Pole Reflection Removal**: Filters out false positive readings from reflective surfaces

**Demo Video:**
![Real-Time Laser Scan Filtering](/Videos/laserScan.mp4)

---

### 3. 🗺️ Localisation Challenge Simulation
A comprehensive navigation and localization system implementing AMCL (Adaptive Monte Carlo Localization) for precise robot positioning.

**System Architecture:**
- **`nav2_map_server`**: Loads and publishes pre-saved environmental maps
- **`nav2_amcl`**: Performs robot localization using LiDAR scan data correlation
- **`nav2_lifecycle_manager`**: Manages node lifecycle and coordination
- **Transform Chain**: Maintains `map ⟶ odom ⟶ base_link ⟶ laser` transformations

**Localization Process:**
1. AMCL compares real-time `/scan` data with the static map
2. Publishes the `map ⟶ odom` transformation for global positioning
3. Obstacle points are computed in LiDAR frame and transformed to map coordinates
4. Markers are created at absolute positions to represent detected obstacles

**Demo Video:**
![Localisation Challenge](Videos/Localization.mp4)

---

### 4. 🚪 Knock Knock Challenge
An intelligent door detection and navigation system that enables autonomous passage through doorways.

**Smart Door Detection:**
- **Laser Scan Analysis**: Continuously monitors for door presence using `/scan` data
- **Distance Threshold Monitoring**: Detects when door distance increases beyond predetermined threshold
- **Door Opening Detection**: Waits for complete door opening before initiating movement
- **Collision Prevention**: Stops movement when obstacles appear within 0.5m (e.g., walls in next room)

**Visualization Features:**
- Real-time laser scan data display in RViz
- Visual representation of door opening process
- Dynamic obstacle detection markers

**Demo Video:**
![Knock Knock Challenge](Videos/KnockKnock.mp4)

---

## 📊 Additional Project Demonstrations

### Navigation System in Action
![Navigation Demo](Videos/Localization.mp4)

## 🛠️ Setup and Installation

Clone the repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/artnie/rpwr-assignments.git
cd rpwr-assignments
```

## 🐳 Run with Docker

Install the Docker Compose Plugin:
- **Linux** (Docker Compose Plugin): [Installation Guide](https://docs.docker.com/compose/install/linux/#install-using-the-repository)
- **Windows** (Docker Desktop): [Installation Guide](https://docs.docker.com/desktop/install/windows-install/)
- **Mac** (Docker Desktop): [Installation Guide](https://docs.docker.com/desktop/install/mac-install/)

Start the Jupyter Notebook environment:

```bash
docker compose -f binder/docker-compose.yml up
```

**Windows Troubleshooting**: For WSL-related issues, refer to the [WSL Installation Guide](https://github.com/IntEL4CoRo/cram_teaching/blob/main/docs/install_wsl.md)

## 📁 Project Structure

```
rpwr-assignments/
├── 01_git-linux-python/     # Git, Linux, and Python fundamentals
├── 02_coordinates-tf/        # Coordinate transformations
├── 03_ros/                   # ROS basics and turtle party
├── 04_navigation/            # Navigation algorithms
├── internship/               # Advanced internship projects
│   ├── Collision Avoidance/  # Lane keeping and collision detection
│   └── src/                  # Core implementation files
├── Videos/                   # Project demonstration videos
└── binder/                   # Docker configuration
```

## 🎥 Video Gallery

All demonstration videos are available in the `Videos/` directory:
- `/Videos/LanekeepingAssistant.mp4` - Lane Keep Assistant Robot
- `/Videos/laserScan.mp4` - Real-Time Laser Scan Filtering
- `/Videos/Localization.mp4` - Localisation Challenge
- `/Videos/KnockKnock.mp4` - Knock Knock Challenge
- `/Videos/LinekeepingAssistant.mp4` - Navigation System Demo

## 🏆 Key Achievements

- ✅ **Real-time Performance**: All systems operate at 40Hz for responsive control
- ✅ **Robust Sensor Processing**: Advanced filtering techniques for reliable data
- ✅ **Autonomous Navigation**: Complete collision avoidance and path planning
- ✅ **Dynamic Obstacle Handling**: Real-time detection and response to environmental changes
- ✅ **Comprehensive Visualization**: Full RViz integration for system monitoring

## 🔧 Technologies Used

- **ROS2** - Robot Operating System
- **Python** - Primary programming language
- **AMCL** - Adaptive Monte Carlo Localization
- **Nav2** - Navigation framework
- **LiDAR** - Laser scan sensor processing
- **RViz** - 3D visualization
- **Docker** - Containerized deployment

---

*This project was developed as part of an internship program, demonstrating practical applications of robotic systems in real-world scenarios.*
