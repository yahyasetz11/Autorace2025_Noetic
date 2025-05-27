# TurtleBot3 Autonomous Racing System

A comprehensive ROS-based autonomous racing system for TurtleBot3 robots, featuring behavior trees, computer vision, and advanced navigation capabilities for competitive racing scenarios.

## 🤖 **Contributor**

**Yahya Setiawan**

**范元嘉 Fan, Yuan-Chia**

## 🏁 **Overview**

This project implements a complete autonomous racing system for TurtleBot3 robots capable of:

- **Traffic light detection and response**
- **Dynamic intersection navigation**
- **Construction zone avoidance**
- **Autonomous parking**
- **Level crossing detection**
- **Tunnel navigation with SLAM/AMCL**
- **Lane following with computer vision**

The system uses behavior trees for high-level decision making and multiple action servers for modular robot behaviors.

## 🏗️ **System Architecture**

### Core Components

- **Action Servers**: Modular robot behaviors (lane detection, construction, parking, etc.)
- **Behavior Tree**: High-level mission coordination and sequencing
- **Computer Vision**: HSV-based detection for lanes, traffic lights, and crosswalks
- **SLAM & Navigation**: Mapping and autonomous navigation in tunnels
- **Configuration System**: YAML-based parameter management

### Package Structure

```
src/
├── action_bt/          # Action servers for robot behaviors
├── behaviortree/       # Behavior tree implementation
├── camera/            # Camera handling and image processing
├── config/            # YAML configuration files
├── controller_manual/  # Moving Turtlebot3 with Joystick
├── crosswalk_detector/ # Crosswalk detection system
├── hsv_reconfigure/   # HSV parameter tuning tools
├── launch_file/       # Launch files for system startup
├── maps/              # Directory of maps.pgm and maps.yaml
├── msg_file/          # Custom msg for this workspace
├── scan_debug/        # Debug the LiDAR value
└── yaw_tracker/       # Orientation tracking
```

## 🚀 **Installation**

### Prerequisites

Ensure you have Ubuntu 20.04 with internet connection. The automated installation script will handle ROS Noetic installation if not already present.

### Clone Repository & Submodules

```bash
# Clone the repository
git clone https://github.com/yahyasetz11/Autorace2025_Noetic.git
cd https://github.com/yahyasetz11/Autorace2025_Noetic.git

# Initialize and update submodules (if any)
git submodule update --init --recursive
```

### Install Dependencies

#### Option 1: Automated Installation (Recommended)

```bash
# Make installation script executable
chmod +x install_dependencies.sh

# Run the automated installation
./install_dependencies.sh
```

#### Option 2: Manual Installation

##### System Dependencies

```bash
# Install ROS Noetic packages
sudo apt update
sudo apt install -y \
    ros-noetic-desktop-full \
    ros-noetic-turtlebot3-* \
    ros-noetic-actionlib \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-joy \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-gmapping

# Install additional packages
sudo apt install -y \
    libopencv-dev \
    libyaml-cpp-dev \
    python3-catkin-tools \
    ros-noetic-behaviortree-cpp-v3
```

##### Python Dependencies

```bash
# Install Python requirements
pip3 install -r requirements.txt
```

##### Build the Workspace

```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
```

## 🎮 **Usage**

### Quick Start

```bash
# Source the workspace
source devel/setup.bash

# Export TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch the complete system
roslaunch launch_file run.launch
```

### Individual Components

#### Launch Action Servers Only

```bash
roslaunch launch_file action_bt.launch
```

#### HSV Parameter Tuning

```bash
# For lane detection tuning
roslaunch launch_file hsv_reconfigure.launch mode:=lane

# For traffic light tuning
roslaunch launch_file hsv_reconfigure.launch mode:=traffic_light

# For crosswalk tuning
roslaunch launch_file hsv_reconfigure.launch mode:=cross_walk
```

#### Camera Calibration

```bash
roslaunch launch_file camera_calibration.launch
```

## 🌳 **Behavior Tree Explanation**

The autonomous racing missions are orchestrated through a behavior tree defined in `src/behaviortree/xml/bt.xml`. The system executes six sequential missions:

### Mission Sequence Overview

```xml
<BehaviorTree ID="LaneDetectBT">
    <Sequence name="BehaviorTree">
        <!-- Mission 0: Traffic Light -->
        <!-- Mission 1: Intersection -->
        <!-- Mission 2: Construction -->
        <!-- Mission 3: Parking -->
        <!-- Mission 4: Level Crossing -->
        <!-- Mission 5: Tunnel -->
        <!-- Mission 6: Finish -->
    </Sequence>
</BehaviorTree>
```

### Node Types and Parameters

#### LaneDetect Node

**Purpose**: Lane following with various modes and conditions

```xml
<LaneDetect mode="center" sign="" duration="5.75" speed="0.20"/>
```

**Parameters**:

- `mode`: `center`, `left`, `right`, `intersection`, `cross_level`, `just-turn-left`, `just-turn-right`
- `sign`: Target sign to detect for completion
- `duration`: Time-based completion (seconds)
- `speed`: Override default speed (m/s)

#### TrafficLight Node

**Purpose**: Detect and respond to traffic lights

```xml
<TrafficLight timeout="20.0"/>
```

#### Construction Node

**Purpose**: Navigate around construction zones

```xml
<Construction mode="ramp_2"/>
```

**Modes**: `rounded`, `ramp`, `ramp_2`

#### Parking Node

**Purpose**: Autonomous parking maneuvers

```xml
<Parking mode="dynamic"/>
```

#### TunnelNav Node

**Purpose**: SLAM-based tunnel navigation

```xml
<TunnelNav mode="offline"/>
```

#### Utility Nodes

- `Rotate`: Precise rotation control
- `Move`: Distance-based movement
- `Align`: Wall alignment

## 🏆 **Mission Details**

### Mission 0: Traffic Light Detection

- Detects red/green traffic lights using HSV color filtering
- Waits for green light confirmation before proceeding
- **Success Condition**: Green light detected and confirmed

### Mission 1: Intersection Navigation

- Dynamic intersection handling with sign detection
- Supports left/right turn decisions based on visual signs
- **Success Condition**: Successfully navigated through intersection

### Mission 2: Construction Zone

- Avoids construction obstacles using multiple strategies
- Supports rounded and ramp-based navigation patterns
- **Success Condition**: Obstacle successfully bypassed

### Mission 3: Autonomous Parking

- Dynamic parking spot detection and maneuvering
- Supports left, right, and dynamic parking modes
- **Success Condition**: Successfully parked and exited

### Mission 4: Level Crossing

- Crosswalk detection using red stripe recognition
- Stops at crosswalk and proceeds when clear
- Complex multi-turn navigation sequence
- **Success Condition**: Crosswalk safely traversed

### Mission 5: Tunnel Navigation

- SLAM-based mapping and autonomous navigation
- Supports both online (mapping) and offline (localization) modes
- **Success Condition**: Successfully navigated to tunnel exit

### Mission 6: Finish Line

- Final approach to race completion
- **Success Condition**: Reached finish line

## ⚙️ **Configuration**

### Key Configuration Files

#### Lane Detection (`src/config/color_lane.yaml`)

```yaml
white_lane:
  hue: { low: 53, high: 179 }
  saturation: { low: 0, high: 66 }
  value: { low: 191, high: 255 }

yellow_lane:
  hue: { low: 20, high: 78 }
  saturation: { low: 30, high: 255 }
  value: { low: 147, high: 255 }
```

#### Speed Control (`src/config/speed_conf.yaml`)

```yaml
max_linear_speed: 0.12
max_angular_speed: 1.0
steering_sensitivity: 3.0
pid_kp: 8.0
pid_ki: 0
pid_kd: 1.6
```

#### Traffic Light Detection (`src/config/traffic_light_param.yaml`)

```yaml
traffic_light:
  roi_x: 162
  roi_y: 140
  roi_width: 15
  roi_height: 19
  red_threshold: 100
  green_threshold: 1
```

### Parameter Tuning

Use the HSV reconfiguration tool for real-time parameter adjustment:

```bash
roslaunch launch_file hsv_reconfigure.launch mode:=<lane|traffic_light|cross_walk>
```

## 🔧 **Troubleshooting**

### Common Issues

#### Action Server Connection Failed

```bash
# Check if action servers are running
rostopic list | grep -E "(goal|result|feedback)"

# Restart action servers
roslaunch launch_file action_bt.launch
```

#### Poor Lane Detection

1. Adjust HSV parameters using `hsv_reconfigure`
2. Check camera calibration
3. Verify lighting conditions

#### Navigation Issues

```bash
# Check TF tree
rosrun tf view_frames

# Verify sensor data
rostopic echo /scan
rostopic echo /odom
```

#### RViz Can't Get the URDF

This issue mostly caused by unsynchronized time between Turtlebot3 and Laptop.

#### At Laptop Side

```bash
# Install ntp
sudo apt install ntp

# Setting the server ntp
sudo nano /etc/ntp.conf
```

And makesure the ntp.conf should be like this (Note : change the 192.168.0.0 as your Network IP)

```conf
# pool 0.ubuntu.pool.ntp.org iburst
# pool 1.ubuntu.pool.ntp.org iburst
# pool 2.ubuntu.pool.ntp.org iburst
# pool 3.ubuntu.pool.ntp.org iburst
server 127.127.1.0
fudge 127.127.1.0 stratum 10

restrict 192.168.0.0 mask 255.255.255.0 nomodify notrap
# Use Ubuntu's ntp server as a fallback.
# pool ntp.ubuntu.com
```

Then restart the NTP system

```bash
sudo systemctl restart ntp
sudo systemctl enable ntp
```

2. At Turtlebot3 Side

```bash
# Install ntp
sudo apt install ntp

# Setting the server ntp
sudo nano /etc/ntp.conf
```

And makesure the ntp.conf should be like this (note: change the 192.168.0.133 to your Laptop IP)

```conf
# pool 0.ubuntu.pool.ntp.org iburst
# pool 1.ubuntu.pool.ntp.org iburst
# pool 2.ubuntu.pool.ntp.org iburst
# pool 3.ubuntu.pool.ntp.org iburst
server 192.168.0.133 iburst

# Use Ubuntu's ntp server as a fallback.
# pool ntp.ubuntu.com
```

Then restart the NTP system

```bash
sudo systemctl restart ntp
sudo systemct
```

Then, check the NTP is it already connected or not by

```bash
ntpq -p
```

It should appear like this

```bash
ubuntu@ubuntu:~$ ntpq -p
     remote           refid      st t when poll reach   delay   offset  jitter
==============================================================================
 192.168.0.133   LOCAL(0)        11 u    -   64    1   17.883    4.901   7.462
ubuntu@ubuntu:~$ ntpq -p
     remote           refid      st t when poll reach   delay   offset  jitter
==============================================================================
*192.168.0.133   LOCAL(0)        11 u    6   64    1    2.775   -0.766   2.909
ubuntu@ubuntu:~$ date
Mon 19 May 2025 07:21:17 PM KST
```

### Debug Commands

```bash
# View active nodes
rosnode list

# Monitor topics
rostopic list
rostopic echo /cmd_vel

# Check transforms
rosrun tf tf_echo map base_link
```

## 📊 **Performance Metrics**

The system is optimized for competitive racing with:

- **Lane detection**: 10-30 FPS depending on processing complexity
- **Response time**: depends on the connection
- **Navigation accuracy**: ±5cm for precise maneuvers
- **Success rate**: >95% in controlled environments

## 📄 **License**

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 **Acknowledgments**

- ROBOTIS for TurtleBot3 platform and packages
- BehaviorTree.CPP community for the behavior tree framework
- OpenCV community for computer vision capabilities
- ROS community for the robotic middleware

---

For additional support or questions, please open an issue in this repository.
