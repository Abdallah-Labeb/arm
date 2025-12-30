# Color Sorting Robotic Arm - ROS1 Project

## مشروع الروبوتيكس والأنظمة الذكية
### Robotic Arm for Color-Based Cube Sorting Using ROS1

---

## 📋 Project Description

This project implements an intelligent robotic system in a simulation environment using a 5-DOF (Degrees of Freedom) robotic arm equipped with a camera and gripper. The robot autonomously:

1. **Detects** colored cubes (red, blue, green) placed in front of it
2. **Estimates** their 3D positions using computer vision and TF transformations
3. **Plans** motion trajectories using inverse kinematics
4. **Picks** each cube using the gripper
5. **Sorts** cubes into designated color zones

The entire system is implemented using **ROS1 (Robot Operating System)**, **Gazebo** for physics simulation, and **RViz** for visualization.

---

## 🎯 Project Objectives

✅ Build a 5-DOF robotic arm model using URDF/Xacro  
✅ Create a Gazebo simulation environment with table and colored cubes  
✅ Implement color detection using OpenCV  
✅ Convert 2D camera detections to 3D coordinates using TF  
✅ Perform complete pick-and-place sorting pipeline  
✅ Integrate all components into a complete ROS1 project  

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                        │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐       │
│  │  5-DOF Arm  │  │ Colored Cubes│  │   Camera    │       │
│  │  + Gripper  │  │ (R, G, B)    │  │   Sensor    │       │
│  └─────────────┘  └──────────────┘  └─────────────┘       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                      ROS1 NODES                             │
│  ┌──────────────────┐  ┌─────────────────┐                │
│  │ Color Detector   │  │ Position        │                │
│  │ (OpenCV)         │─▶│ Estimator (TF)  │                │
│  └──────────────────┘  └─────────────────┘                │
│           │                     │                           │
│           └─────────┬───────────┘                           │
│                     ▼                                       │
│  ┌──────────────────────────────────────┐                 │
│  │   Sorting Controller (Main Logic)    │                 │
│  └──────────────────────────────────────┘                 │
│           │                     │                           │
│           ▼                     ▼                           │
│  ┌─────────────────┐  ┌─────────────────┐                │
│  │ Motion Planner  │  │ Gripper         │                │
│  │ (IK + Trajectry)│  │ Controller      │                │
│  └─────────────────┘  └─────────────────┘                │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                    ┌───────────────┐
                    │  RVIZ (Visual)│
                    └───────────────┘
```

---

## 📁 Project Structure

```
color_sorting_arm/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package dependencies
├── README.md                   # This file
│
├── urdf/                       # Robot models
│   ├── arm.urdf.xacro         # Main robot URDF
│   └── arm.gazebo.xacro       # Gazebo plugins & sensors
│
├── worlds/                     # Gazebo worlds
│   └── sorting_world.world    # Simulation environment
│
├── config/                     # Configuration files
│   ├── controllers.yaml       # Joint controllers config
│   └── sorting_arm.rviz       # RViz visualization config
│
├── launch/                     # Launch files
│   ├── complete_system.launch # Launch everything
│   ├── gazebo.launch          # Launch Gazebo only
│   ├── rviz.launch            # Launch RViz only
│   └── nodes.launch           # Launch ROS nodes only
│
├── scripts/                    # Python nodes
│   ├── color_detector.py      # OpenCV color detection
│   ├── position_estimator.py  # 2D→3D coordinate transform
│   ├── motion_planner.py      # Inverse kinematics & motion
│   ├── gripper_controller.py  # Gripper open/close control
│   └── sorting_controller.py  # Main sorting orchestrator
│
└── msg/                        # Custom ROS messages
    ├── DetectedObject.msg     # 2D detected object
    ├── DetectedObjectArray.msg
    ├── Object3D.msg           # 3D object with position
    └── Object3DArray.msg
```

---

## 🔧 Installation & Dependencies

### Prerequisites

- **Ubuntu 20.04** (recommended)
- **ROS Noetic** (ROS1)
- **Python 3**
- **Gazebo 11**

### Required ROS Packages

```bash
sudo apt update
sudo apt install -y \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-position-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-xacro \
    python3-opencv \
    python3-numpy
```

### Setup Catkin Workspace

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Copy this project to src folder
cp -r /path/to/color_sorting_arm .

# Build the workspace
cd ~/catkin_ws
catkin_make

# Source the workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Make Python Scripts Executable

```bash
cd ~/catkin_ws/src/color_sorting_arm/scripts
chmod +x *.py
```

---

## 🚀 Usage

### Method 1: Launch Complete System

This launches Gazebo, RViz, and all ROS nodes together:

```bash
roslaunch color_sorting_arm complete_system.launch
```

### Method 2: Launch Components Separately

**Terminal 1 - Gazebo Simulation:**
```bash
roslaunch color_sorting_arm gazebo.launch
```

**Terminal 2 - RViz Visualization:**
```bash
roslaunch color_sorting_arm rviz.launch
```

**Terminal 3 - ROS Nodes:**
```bash
roslaunch color_sorting_arm nodes.launch
```

---

## 🎮 How It Works

### 1. **Color Detection** (`color_detector.py`)
- Subscribes to camera image topic: `/arm_camera/image_raw`
- Uses OpenCV HSV color segmentation to detect red, blue, and green cubes
- Publishes detected objects with pixel coordinates to `/detected_objects`
- Publishes annotated image to `/detection_image`

### 2. **Position Estimation** (`position_estimator.py`)
- Subscribes to `/detected_objects` and `/arm_camera/camera_info`
- Converts 2D pixel coordinates to 3D world coordinates
- Uses TF transformations (`camera_optical_frame` → `base_link`)
- Publishes 3D positions to `/object_positions`

### 3. **Motion Planning** (`motion_planner.py`)
- Implements inverse kinematics for 5-DOF arm
- Plans trajectories for pick-and-place operations
- Publishes joint commands to position controllers
- Functions: `move_to_position()`, `pick_object()`, `place_object()`

### 4. **Gripper Control** (`gripper_controller.py`)
- Controls gripper fingers (prismatic joints)
- Subscribes to `/gripper_command` (Bool: True=Open, False=Close)
- Publishes to gripper joint controllers

### 5. **Sorting Controller** (`sorting_controller.py`)
- Main orchestration node
- Coordinates all other nodes
- Implements sorting logic:
  1. Wait for object detection
  2. For each detected object:
     - Open gripper
     - Move to object position
     - Pick object (close gripper)
     - Move to corresponding color zone
     - Place object (open gripper)
  3. Return to home position

---

## 🎨 Color Zones

The cubes are sorted into designated zones on the table:

| Color  | Zone Position (x, y, z) | Description |
|--------|-------------------------|-------------|
| 🔴 Red  | (0.2, 0.3, 0.85)       | Right zone  |
| 🔵 Blue | (0.2, 0.0, 0.85)       | Center zone |
| 🟢 Green| (0.2, -0.3, 0.85)      | Left zone   |

---

## 📊 ROS Topics

### Published Topics:
- `/detected_objects` - Detected cubes in pixel coordinates
- `/detection_image` - Camera image with detection annotations
- `/object_positions` - 3D positions of detected cubes
- `/gripper_command` - Gripper open/close commands
- `/sorting_arm/joint_states` - Current joint positions
- Controller command topics for each joint

### Subscribed Topics:
- `/arm_camera/image_raw` - Camera RGB image
- `/arm_camera/camera_info` - Camera calibration info

---

## 🔍 Monitoring & Debugging

### View Camera Feed with Detections
```bash
rosrun image_view image_view image:=/detection_image
```

### Monitor Topics
```bash
# List all topics
rostopic list

# Echo detected objects
rostopic echo /detected_objects

# Echo 3D positions
rostopic echo /object_positions

# Monitor joint states
rostopic echo /sorting_arm/joint_states
```

### Check TF Tree
```bash
rosrun tf view_frames
evince frames.pdf
```

### RQT Graph
```bash
rqt_graph
```

---

## ⚙️ Configuration

### Adjust Color Detection Ranges
Edit `scripts/color_detector.py`:
```python
self.color_ranges = {
    'red': [...],    # Modify HSV ranges
    'blue': [...],
    'green': [...]
}
```

### Modify Sorting Zones
Edit `scripts/sorting_controller.py`:
```python
self.sorting_zones = {
    'red': (x, y, z),
    'blue': (x, y, z),
    'green': (x, y, z)
}
```

### Tune PID Controllers
Edit `config/controllers.yaml`:
```yaml
joint1_position_controller:
  type: position_controllers/JointPositionController
  joint: joint1
  pid: {p: 100.0, i: 0.01, d: 10.0}  # Modify values
```

---

## 🐛 Troubleshooting

### Problem: Gazebo doesn't start or crashes
**Solution:** 
```bash
killall gzserver gzclient
roslaunch color_sorting_arm gazebo.launch
```

### Problem: Controllers fail to load
**Solution:** Check controller spawner:
```bash
rosrun controller_manager controller_manager list
```

### Problem: Camera not publishing images
**Solution:** Check Gazebo camera plugin:
```bash
rostopic hz /arm_camera/image_raw
```

### Problem: TF transform errors
**Solution:** Check TF tree:
```bash
rosrun tf tf_echo base_link camera_optical_frame
```

### Problem: Inverse kinematics not reaching target
**Solution:** 
- Target may be out of workspace
- Adjust link lengths in `motion_planner.py`
- Check joint limits in URDF

---

## 📈 Future Improvements

- [ ] Add depth camera for better 3D position estimation
- [ ] Implement MoveIt! for advanced motion planning
- [ ] Add obstacle avoidance
- [ ] Support more colors/object shapes
- [ ] Implement machine learning for object recognition
- [ ] Add force/torque sensors for better grasping
- [ ] Multi-robot coordination

---

## 👥 Team Information

**Course:** Robotics and Intelligent Systems  
**Project:** Color-Based Cube Sorting Using ROS1  
**Maximum Team Size:** 6 students  

**Team Members:**
1. [Your Name]
2. [Team Member 2]
3. [Team Member 3]
4. [Team Member 4]
5. [Team Member 5]
6. [Team Member 6]

---

## 📚 References

- [ROS Wiki](http://wiki.ros.org/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [OpenCV Python Tutorials](https://docs.opencv.org/master/d6/d00/tutorial_py_root.html)
- [ROS Control](http://wiki.ros.org/ros_control)

---

## 📝 License

MIT License - Feel free to use this project for educational purposes.

---

## 🎓 Learning Outcomes

By completing this project, you will learn:

1. ✅ **URDF/Xacro modeling** - Create robot descriptions
2. ✅ **Gazebo simulation** - Build physics-based environments
3. ✅ **ROS architecture** - Design multi-node systems
4. ✅ **Computer vision** - OpenCV color detection
5. ✅ **TF transformations** - Coordinate frame conversions
6. ✅ **Inverse kinematics** - Motion planning algorithms
7. ✅ **ROS control** - Joint controllers and trajectories
8. ✅ **System integration** - Combine multiple components

---

## 🎉 Success!

If everything works correctly, you should see:
1. Gazebo simulation with robotic arm and colored cubes
2. RViz showing robot model and camera feed with detections
3. Terminal logs showing detection and sorting progress
4. Arm autonomously picking and sorting cubes by color

**Good luck with your project! 🤖🎨**

---

## 📧 Support

For questions or issues:
- Check ROS Answers: https://answers.ros.org/
- Review Gazebo Forums: https://community.gazebosim.org/
- Contact your course instructor

---

**Last Updated:** December 2025
