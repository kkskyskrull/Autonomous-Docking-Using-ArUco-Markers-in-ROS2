https://github.com/kkskyskrull/Autonomous-Docking-Using-ArUco-Markers-in-ROS2.githttps://github.com/kkskyskrull/Autonomous-Docking-Using-ArUco-Markers-in-ROS2.git# ROS2 Autonomous Docking with ArUco Markers

## Project Overview
ROS2-based solution for autonomously docking a robot using ArUco markers. This project includes robot simulation, marker detection, and docking logic.

## Repository Structure

ros2_ws/
├── README.md
├── LICENSE
├── src/
│ ├── robot_description/ # Robot URDF, RViz configs, launch files
│ └── aruco_detector/ # ArUco detection and virtual camera nodes


## Packages

### 1. robot_description
- **Description**: Contains URDF model of differential drive robot (diffbot)
- **Files**:
  - `urdf/diffbot.urdf` - Robot URDF with camera
  - `launch/display_diffbot.launch.py` - Launch file for visualization
  - `rviz/diffbot.rviz` - RViz configuration

### 2. aruco_detector
- **Description**: ArUco marker detection and virtual camera simulation
- **Nodes**:
  - `aruco_detector_node` - Detects ArUco markers and publishes poses
  - `virtual_camera_node` - Simulates camera feed for testing

## Installation

### Prerequisites
- ROS2 Humble (on Ubuntu 22.04.5 LTS)
- OpenCV with ArUco support

### Setup
```bash
# Clone repository
git clone <https://github.com/kkskyskrull/Autonomous-Docking-Using-ArUco-Markers-in-ROS2.git>
cd ros2_ws

# Install dependencies
sudo apt update
sudo apt install ros-${ROS_DISTRO}-cv-bridge
pip install opencv-contrib-python

# Build packages
colcon build
source install/setup.bash
