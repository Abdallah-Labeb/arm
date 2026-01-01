# Color Sorting Arm - Quick Start Guide

## Prerequisites

- Ubuntu 20.04 LTS
- ROS Noetic

## Installation Steps

### 1. Install ROS Noetic (if not installed)

```bash
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Source ROS
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Dependencies

```bash
# Install required packages
sudo apt install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-cv-bridge \
    python3-opencv \
    python3-pip

# Install Python dependencies
pip3 install numpy opencv-python
```

### 3. Setup Workspace

```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the project (or copy it here)
# If copying: cp -r /path/to/color_sorting_arm .

# Build
cd ~/catkin_ws
catkin_make

# Source workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Make Python scripts executable
chmod +x ~/catkin_ws/src/color_sorting_arm/scripts/*.py
```

### 4. Run the System

#### Option A: Run Complete System (All at once)

```bash
roslaunch color_sorting_arm complete_system.launch
```

#### Option B: Run Step by Step (Recommended for debugging)

**Terminal 1 - Launch Gazebo and Robot:**
```bash
roslaunch color_sorting_arm gazebo.launch
```

Wait for Gazebo to fully load (you should see the robot on the table with colored cubes).

**Terminal 2 - Launch RViz:**
```bash
roslaunch color_sorting_arm rviz.launch
```

**Terminal 3 - Launch Processing Nodes:**
```bash
roslaunch color_sorting_arm nodes.launch
```

## Verification

### Check Topics

```bash
# List all topics
rostopic list

# Check camera is publishing
rostopic hz /arm_camera/image_raw

# Check joint states
rostopic echo /sorting_arm/joint_states -n 1

# Check detected objects
rostopic echo /detected_objects
```

### Check TF Tree

```bash
# View TF tree
rosrun rqt_tf_tree rqt_tf_tree
```

### Manual Joint Control (Testing)

```bash
# Move joint 1 (base rotation)
rostopic pub /sorting_arm/joint1_position_controller/command std_msgs/Float64 "data: 0.5"

# Move joint 2 (shoulder)
rostopic pub /sorting_arm/joint2_position_controller/command std_msgs/Float64 "data: 0.3"

# Open gripper
rostopic pub /sorting_arm/gripper_left_position_controller/command std_msgs/Float64 "data: 0.02"
rostopic pub /sorting_arm/gripper_right_position_controller/command std_msgs/Float64 "data: 0.02"

# Close gripper
rostopic pub /sorting_arm/gripper_left_position_controller/command std_msgs/Float64 "data: 0.005"
rostopic pub /sorting_arm/gripper_right_position_controller/command std_msgs/Float64 "data: 0.005"
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        GAZEBO SIMULATION                        │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────────┐    │
│  │  Robot  │  │  Table  │  │  Cubes  │  │  Camera Plugin  │    │
│  └─────────┘  └─────────┘  └─────────┘  └─────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         ROS NODES                               │
│                                                                 │
│  ┌────────────────┐    ┌───────────────────┐                   │
│  │ Color Detector │───▶│ Position Estimator│                   │
│  │  (OpenCV HSV)  │    │   (TF Transform)  │                   │
│  └────────────────┘    └───────────────────┘                   │
│                              │                                  │
│                              ▼                                  │
│  ┌────────────────┐    ┌───────────────────┐                   │
│  │Gripper Control │◀───│ Sorting Controller│                   │
│  │               │    │   (Pick & Place)  │                   │
│  └────────────────┘    └───────────────────┘                   │
└─────────────────────────────────────────────────────────────────┘
```

## Troubleshooting

### "No module named color_sorting_arm.msg"
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Robot not appearing in Gazebo
```bash
# Check robot description
rosparam get /robot_description | head -50
```

### TF transforms not working
```bash
# Check TF tree
rosrun tf2_tools view_frames.py
evince frames.pdf
```

### Camera not showing image
```bash
# Check camera topic
rostopic echo /arm_camera/camera_info -n 1
rosrun image_view image_view image:=/arm_camera/image_raw
```

## File Structure

```
color_sorting_arm/
├── config/
│   ├── controllers.yaml    # PID controller settings
│   └── sorting_arm.rviz    # RViz configuration
├── launch/
│   ├── complete_system.launch  # All-in-one launch
│   ├── gazebo.launch          # Gazebo + robot
│   ├── nodes.launch           # Python nodes
│   └── rviz.launch            # RViz visualization
├── msg/
│   ├── DetectedObject.msg     # 2D detection
│   ├── DetectedObjectArray.msg
│   ├── Object3D.msg           # 3D position
│   └── Object3DArray.msg
├── scripts/
│   ├── color_detector.py      # HSV color detection
│   ├── gripper_controller.py  # Gripper open/close
│   ├── position_estimator.py  # 2D to 3D conversion
│   └── sorting_controller.py  # Main control logic
├── urdf/
│   ├── arm.urdf.xacro        # Robot model
│   └── arm.gazebo.xacro      # Gazebo plugins
└── worlds/
    └── sorting_world.world    # Simulation environment
```
