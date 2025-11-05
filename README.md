# AutoSLAM - Integrated SLAM System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)

An integrated ROS2 package that combines motor control, camera streaming, and SLAM capabilities for autonomous robotics applications. AutoSLAM provides flexible deployment architectures to optimize performance between robot-side and laptop-side processing.

## 🚀 Features

- **Dual Architecture Support**: Choose between distributed processing and edge computing configurations
- **Complete SLAM Integration**: Built on RTAB-Map for robust simultaneous localization and mapping
- **Motor Control**: Integrated motor control with teleop support
- **Camera Integration**: Seamless OAK-D camera integration for visual SLAM
- **Flexible Deployment**: Configure processing distribution based on your hardware constraints
- **Real-time Performance**: Optimized for real-time robotics applications

## 📋 Table of Contents

- [Installation](#installation)
- [Architecture Overview](#architecture-overview)
- [Usage](#usage)
- [Configuration](#configuration)
- [Dependencies](#dependencies)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## 🔧 Installation

### Prerequisites

- ROS2 Jazzy or later
- Ubuntu 24.04 or compatible
- OAK-D camera
- Robot with motor control hardware

### Dependencies

This package depends on the following packages:
- `robot_base` - Motor control and teleop functionality
- `oakd_driver` - OAK-D camera driver
- `map_builder` - RTAB-Map integration
- `rtabmap_ros` - RTAB-Map ROS2 bindings

### Build Instructions

```bash
# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository (or copy the packages)
# Ensure you have all required packages: autoslam, robot_base, oakd_driver, map_builder

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select robot_base oakd_driver map_builder autoslam
source install/setup.bash
```

## 🏗️ Architecture Overview

AutoSLAM supports two main deployment architectures:

### 1. Distributed Processing Architecture
**Best for**: Limited onboard computing, high-bandwidth network connection
```
Robot Side:          Laptop Side:
├── Motor Control    ├── Teleop Control
├── Camera Stream    ├── RTAB-Map Processing
└── Data Streaming   └── Visualization
```

### 2. Edge Computing Architecture
**Best for**: Powerful onboard computing, low-bandwidth or intermittent connectivity
```
Robot Side:              Laptop Side:
├── Motor Control        ├── Teleop Control
├── Camera Processing    └── Map Viewing (optional)
├── RTAB-Map SLAM        
└── Autonomous Operation 
```

## 🎮 Usage

### Distributed Processing Mode

Perfect when you have a powerful laptop and want to offload processing from the robot.

**Robot Side:**
```bash
ros2 launch autoslam distributed_processing_robot.launch.py
```

**Laptop Side:**
```bash
ros2 launch autoslam distributed_processing_laptop.launch.py
```

### Edge Computing Mode

Ideal for autonomous operation with minimal dependency on external processing.

**Robot Side:**
```bash
ros2 launch autoslam edge_computing_robot.launch.py
```

**Laptop Side (optional):**
```bash
ros2 launch autoslam edge_computing_laptop.launch.py
```

### Launch Arguments

#### Common Arguments
- `camera_name` (default: 'oak'): Name of the camera node
- `max_duty` (default: '1000'): Maximum duty cycle for motor control
- `robot_namespace` (default: 'robot'): Namespace for robot nodes

#### Distributed Processing Specific
- `robot_ip` (default: '192.168.1.100'): IP address of the robot
- `use_rtabmap_viz` (default: 'true'): Enable RTAB-Map visualization

#### Edge Computing Specific
- `enable_rtabmap_viz` (default: 'false'): Enable visualization on robot
- `camera_params_file`: Camera configuration file optimized for RTAB-Map

### Example Usage with Custom Parameters

```bash
# Distributed processing with custom robot IP
ros2 launch autoslam distributed_processing_laptop.launch.py robot_ip:=192.168.1.150

# Edge computing with visualization enabled
ros2 launch autoslam edge_computing_robot.launch.py enable_rtabmap_viz:=true

# Custom motor duty cycle
ros2 launch autoslam distributed_processing_robot.launch.py max_duty:=1500
```

## ⚙️ Configuration

### Network Configuration (Distributed Processing)

For distributed processing, ensure proper network configuration between robot and laptop:

1. **Set up ROS_DOMAIN_ID** (same on both devices):
```bash
export ROS_DOMAIN_ID=42
```

2. **Configure network discovery**:
```bash
# On robot
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# On laptop  
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

### Camera Configuration

Camera parameters can be customized by modifying the configuration files in the `oakd_driver` package:
- `camera.yaml` - General camera configuration
- `rgbd.yaml` - RGBD-optimized for SLAM

### Motor Control

Motor control parameters are configured in the `robot_base` package. Adjust `max_duty` based on your motor specifications.

### AutoSLAM Configuration

Currently, the autoslam package uses launch parameters for configuration rather than separate config files. All settings are passed as launch arguments to maintain simplicity and flexibility.

## 📦 Package Structure

```
autoslam/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package manifest
├── README.md                   # This file
├── PACKAGE_RENAMING_SUMMARY.md # Migration notes
├── launch/                     # Launch files
│   ├── distributed_processing_robot.launch.py
│   ├── distributed_processing_laptop.launch.py
│   ├── edge_computing_robot.launch.py
│   └── edge_computing_laptop.launch.py
├── config/                     # Configuration files (currently empty)
└── scripts/                    # Utility scripts (currently empty)
```

## 🔌 Dependencies

### Core ROS2 Packages
- `rclcpp` / `rclpy` - ROS2 client libraries
- `sensor_msgs` - Sensor message types
- `geometry_msgs` - Geometry message types
- `nav_msgs` - Navigation message types

### SLAM Dependencies
- `rtabmap_slam` - RTAB-Map SLAM algorithm
- `rtabmap_viz` - RTAB-Map visualization
- `rtabmap_conversions` - Message conversions
- `rtabmap_util` - Utilities

### Hardware Dependencies
- `robot_base` - Motor control interface
- `oakd_driver` - OAK-D camera driver
- `map_builder` - RTAB-Map integration

### Visualization
- `rviz2` - 3D visualization
- `image_transport` - Image streaming

## 🔍 Troubleshooting

### Common Issues

1. **Camera not detected**
   - Ensure OAK-D camera is properly connected
   - Check USB permissions: `sudo usermod -a -G plugdev $USER`
   - Verify camera configuration in `oakd_driver/config/`

2. **Motor control not responding**
   - Check motor driver connections
   - Verify motor controller parameters in `robot_base`
   - Ensure proper permissions for hardware access

3. **Network connectivity issues (Distributed mode)**
   - Verify robot and laptop are on the same network
   - Check firewall settings
   - Ensure ROS_DOMAIN_ID matches on both devices

4. **RTAB-Map performance issues**
   - Reduce camera resolution in configuration
   - Adjust RTAB-Map parameters in `map_builder/config/`
   - Consider switching to edge computing mode

### Debug Commands

```bash
# Check if all packages are found
ros2 pkg list | grep -E "(robot_base|autoslam|oakd_driver|map_builder)"

# Monitor topics
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /oak/rgb/image_raw

# Check node status
ros2 node list
ros2 node info /motor_node

# Test launch files
ros2 launch autoslam distributed_processing_robot.launch.py --help
```

## 🎯 Performance Optimization

### For Resource-Constrained Robots
- Use distributed processing architecture
- Disable unnecessary visualization
- Optimize camera settings for lower bandwidth

### For Autonomous Operation
- Use edge computing architecture
- Enable sufficient onboard processing power
- Configure robust SLAM parameters

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Setup

```bash
# Clone development version
git clone <repository-url>
cd autoslam

# Install development dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build in development mode
colcon build --symlink-install
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [RTAB-Map](http://introlab.github.io/rtabmap/) for the SLAM algorithm
- [DepthAI](https://docs.luxonis.com/) for OAK-D camera support
- [ROS2](https://docs.ros.org/en/jazzy/) community for the robotics framework

## 📞 Support

For support and questions:
- Create an issue on GitHub
- Check the [troubleshooting section](#troubleshooting)
- Review the launch file documentation

---

**AutoSLAM** - Bringing autonomous SLAM capabilities to your robotics projects! 🤖