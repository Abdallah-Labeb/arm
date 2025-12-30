# Quick Start Guide

## Installation (5 minutes)

1. **Install ROS Noetic:**
   ```bash
   # Follow: http://wiki.ros.org/noetic/Installation/Ubuntu
   ```

2. **Install dependencies:**
   ```bash
   cd /path/to/color_sorting_arm
   bash setup.sh
   ```

3. **Build workspace:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Test installation:**
   ```bash
   rosrun color_sorting_arm test_installation.py
   ```

## Running the Project (1 command!)

```bash
roslaunch color_sorting_arm complete_system.launch
```

## What You Should See

1. **Gazebo window:** 
   - Robotic arm on a table
   - 6 colored cubes (2 red, 2 blue, 2 green)
   - Colored sorting zones

2. **RViz window:**
   - 3D robot model
   - Camera feed with colored boxes around detected cubes
   - TF frames

3. **Terminal output:**
   - "Color Detector Node initialized"
   - "Position Estimator Node initialized"
   - "Detected X objects"
   - "Sorting red cube..."
   - Progress messages

## Expected Behavior

The robot will automatically:
1. Detect all cubes using camera
2. Calculate their positions
3. Pick each cube one by one
4. Sort them into color-matching zones
5. Return to home position

Total time: ~2-3 minutes per cube

## Troubleshooting

**Problem:** Gazebo doesn't start
```bash
killall gzserver gzclient
roslaunch color_sorting_arm gazebo.launch
```

**Problem:** No objects detected
- Wait 5-10 seconds for camera to initialize
- Check camera view in RViz
- Adjust lighting in Gazebo (View → Transparent/Wireframe)

**Problem:** Arm doesn't move
- Check controllers: `rosrun controller_manager controller_manager list`
- Restart simulation

**Problem:** Build errors
```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
```

## Quick Commands

```bash
# Launch everything
roslaunch color_sorting_arm complete_system.launch

# Launch only simulation
roslaunch color_sorting_arm gazebo.launch

# View camera with detections
rosrun image_view image_view image:=/detection_image

# Monitor detected objects
rostopic echo /detected_objects

# List all topics
rostopic list

# Check TF tree
rosrun tf view_frames
```

## Tips for Success

1. **Let it initialize:** Wait 10 seconds after launch before expecting action
2. **Camera view:** The robot needs to "see" the cubes - don't block the camera
3. **Patience:** Each pick-and-place takes time for smooth motion
4. **Recovery:** If something goes wrong, just restart the launch file

## Demo Video Recording

```bash
# Record a bag file
rosbag record -a -O sorting_demo.bag

# Play it back later
rosbag play sorting_demo.bag
```

## Next Steps

- Modify cube positions in `worlds/sorting_world.world`
- Adjust color detection ranges in `scripts/color_detector.py`
- Change sorting zone positions in `scripts/sorting_controller.py`
- Add more cubes or colors
- Improve inverse kinematics algorithm

---

**Need help?** Check the full README.md for detailed documentation.

**Good luck! 🚀🤖**
