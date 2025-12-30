#!/usr/bin/env python3
"""
Test script to verify all components are working
Run this after building the workspace to check the installation
"""

import sys
import importlib

def check_python_packages():
    """Check if required Python packages are installed"""
    print("\n=== Checking Python Packages ===")
    
    packages = {
        'rospy': 'ROS Python client library',
        'cv2': 'OpenCV (python3-opencv)',
        'numpy': 'NumPy (python3-numpy)',
        'sensor_msgs.msg': 'ROS sensor messages',
        'geometry_msgs.msg': 'ROS geometry messages',
        'std_msgs.msg': 'ROS standard messages',
    }
    
    all_ok = True
    for package, description in packages.items():
        try:
            importlib.import_module(package)
            print(f"✓ {package:20s} - OK")
        except ImportError:
            print(f"✗ {package:20s} - MISSING ({description})")
            all_ok = False
    
    return all_ok

def check_ros_environment():
    """Check ROS environment variables"""
    print("\n=== Checking ROS Environment ===")
    
    import os
    
    vars_to_check = {
        'ROS_DISTRO': 'ROS distribution',
        'ROS_MASTER_URI': 'ROS master URI',
        'ROS_PACKAGE_PATH': 'ROS package path',
    }
    
    all_ok = True
    for var, description in vars_to_check.items():
        value = os.environ.get(var)
        if value:
            print(f"✓ {var:20s} = {value}")
        else:
            print(f"✗ {var:20s} - NOT SET ({description})")
            all_ok = False
    
    return all_ok

def check_custom_messages():
    """Check if custom messages are built"""
    print("\n=== Checking Custom Messages ===")
    
    try:
        from color_sorting_arm.msg import DetectedObject, Object3D
        print("✓ Custom messages built successfully")
        return True
    except ImportError as e:
        print(f"✗ Custom messages not found: {e}")
        print("  Run 'catkin_make' in your workspace")
        return False

def main():
    print("="*50)
    print("Color Sorting Arm - Installation Test")
    print("="*50)
    
    results = []
    
    # Check Python packages
    results.append(check_python_packages())
    
    # Check ROS environment
    results.append(check_ros_environment())
    
    # Check custom messages
    results.append(check_custom_messages())
    
    # Summary
    print("\n" + "="*50)
    if all(results):
        print("✓ All checks passed! System ready.")
        print("\nYou can now launch the project:")
        print("  roslaunch color_sorting_arm complete_system.launch")
        return 0
    else:
        print("✗ Some checks failed. Please fix the issues above.")
        print("\nCommon solutions:")
        print("  1. Install missing packages:")
        print("     sudo apt install ros-noetic-<package>")
        print("  2. Source ROS workspace:")
        print("     source /opt/ros/noetic/setup.bash")
        print("     source ~/catkin_ws/devel/setup.bash")
        print("  3. Build workspace:")
        print("     cd ~/catkin_ws && catkin_make")
        return 1
    print("="*50)

if __name__ == '__main__':
    sys.exit(main())
