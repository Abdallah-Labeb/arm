#!/usr/bin/env python3
"""
Sorting Controller Node
Main control node for the color sorting robotic arm.
Handles pick and place operations based on detected object colors.
"""

import rospy
import math
import numpy as np
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from color_sorting_arm.msg import Object3DArray

class SortingController:
    """Main controller for sorting colored cubes by color."""
    
    # Sorting bin locations (x, y, z) in base_link frame
    BIN_LOCATIONS = {
        'red': (0.1, 0.25, 0.08),    # Left bin
        'blue': (0.1, -0.25, 0.08),  # Right bin
        'green': (0.4, 0.0, 0.08)    # Front bin
    }
    
    # Robot arm parameters
    L1 = 0.13   # base_link to link1 + link1 height
    L2 = 0.15   # link2 length (shoulder to elbow)
    L3 = 0.15   # link3 length (elbow to wrist)
    L4 = 0.12   # link4 + link5 + gripper
    
    def __init__(self):
        rospy.init_node('sorting_controller')
        rospy.loginfo("Starting Sorting Controller...")
        
        # State variables
        self.current_joints = [0.0] * 7  # 5 arm joints + 2 gripper
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 
                           'gripper_left_joint', 'gripper_right_joint']
        self.detected_objects = []
        self.is_busy = False
        self.gripper_open = True
        
        # Publishers for joint position commands
        self.joint_pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.joint_pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/sorting_arm/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/object_positions', Object3DArray, self.objects_callback)
        
        # Wait for connections
        rospy.sleep(2.0)
        
        # Move to home position
        self.move_to_home()
        rospy.sleep(2.0)
        
        # Main control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.5), self.control_loop)
        
        rospy.loginfo("Sorting Controller initialized successfully!")
    
    def joint_state_callback(self, msg):
        """Update current joint positions from joint states."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_joints[idx] = msg.position[i]
    
    def objects_callback(self, msg):
        """Receive detected 3D object positions."""
        self.detected_objects = msg.objects
    
    def publish_joint_command(self, joint_name, position):
        """Publish command to a single joint."""
        if joint_name in self.joint_pubs:
            cmd = Float64()
            cmd.data = float(position)
            self.joint_pubs[joint_name].publish(cmd)
    
    def set_joint_positions(self, positions, duration=2.0):
        """
        Set all joint positions and wait for completion.
        positions: [j1, j2, j3, j4, j5]
        """
        # Publish all joint commands
        for i, pos in enumerate(positions):
            self.publish_joint_command(f'joint{i+1}', pos)
        
        # Wait for motion to complete
        rospy.sleep(duration)
    
    def open_gripper(self):
        """Open the gripper."""
        rospy.loginfo("Opening gripper...")
        self.gripper_left_pub.publish(Float64(0.02))
        self.gripper_right_pub.publish(Float64(0.02))
        self.gripper_open = True
        rospy.sleep(0.5)
    
    def close_gripper(self):
        """Close the gripper to grasp object."""
        rospy.loginfo("Closing gripper...")
        self.gripper_left_pub.publish(Float64(0.005))
        self.gripper_right_pub.publish(Float64(0.005))
        self.gripper_open = False
        rospy.sleep(0.5)
    
    def move_to_home(self):
        """Move arm to home/observation position."""
        rospy.loginfo("Moving to home position...")
        # Home position: arm looking forward and slightly down
        home = [0.0, 0.3, -0.5, 0.2, 0.0]
        self.set_joint_positions(home, 2.0)
        self.open_gripper()
    
    def inverse_kinematics(self, x, y, z, pitch=math.pi/2):
        """
        Calculate joint angles to reach target position.
        Uses geometric approach for 5-DOF arm.
        
        Args:
            x, y, z: Target position in base_link frame
            pitch: Desired end-effector pitch angle (default: pointing down)
        
        Returns:
            List of 5 joint angles or None if unreachable
        """
        try:
            # Joint 1: Base rotation
            j1 = math.atan2(y, x)
            
            # Distance in XY plane
            r = math.sqrt(x**2 + y**2)
            
            # Wrist position (accounting for gripper/wrist length)
            wrist_offset = self.L4 * math.cos(pitch)
            z_offset = self.L4 * math.sin(pitch)
            
            r_wrist = r - wrist_offset if r > wrist_offset else 0.01
            z_wrist = z - self.L1 + z_offset
            
            # Distance from shoulder to wrist
            d = math.sqrt(r_wrist**2 + z_wrist**2)
            
            # Check reachability
            max_reach = self.L2 + self.L3
            if d > max_reach * 0.95:
                rospy.logwarn(f"Target may be at edge of reach: d={d:.3f}, max={max_reach:.3f}")
                d = max_reach * 0.95
            
            # Elbow angle using law of cosines
            cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            cos_j3 = np.clip(cos_j3, -1.0, 1.0)
            j3 = math.acos(cos_j3)  # Elbow angle
            
            # Shoulder angle
            alpha = math.atan2(z_wrist, r_wrist)
            beta = math.acos((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d))
            j2 = alpha + beta  # Shoulder lifts up
            
            # Wrist pitch to maintain end-effector orientation
            j4 = pitch - j2 - j3 + math.pi/2
            
            # Wrist roll (keep at 0 for now)
            j5 = 0.0
            
            # Adjust j3 to match URDF convention
            j3 = -(math.pi - j3)
            
            return [j1, j2, j3, j4, j5]
            
        except Exception as e:
            rospy.logerr(f"IK calculation failed: {e}")
            return None
    
    def move_to_position(self, x, y, z, pitch=math.pi/2):
        """Move end-effector to specified position."""
        rospy.loginfo(f"Moving to position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        joints = self.inverse_kinematics(x, y, z, pitch)
        if joints is None:
            rospy.logerr("Failed to calculate IK solution")
            return False
        
        rospy.loginfo(f"Joint angles: {[f'{j:.2f}' for j in joints]}")
        self.set_joint_positions(joints, 2.0)
        return True
    
    def pick_object(self, x, y, z):
        """
        Execute pick operation at given position.
        """
        rospy.loginfo(f"Picking object at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Approach position (above the object)
        approach_z = z + 0.1
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        # Move down to grasp
        if not self.move_to_position(x, y, z + 0.02):
            return False
        rospy.sleep(0.5)
        
        # Close gripper
        self.close_gripper()
        rospy.sleep(0.5)
        
        # Lift object
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        return True
    
    def place_object(self, x, y, z):
        """
        Execute place operation at given position.
        """
        rospy.loginfo(f"Placing object at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Approach position (above the target)
        approach_z = z + 0.1
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        # Move down to place
        if not self.move_to_position(x, y, z + 0.03):
            return False
        rospy.sleep(0.5)
        
        # Open gripper to release
        self.open_gripper()
        rospy.sleep(0.5)
        
        # Lift away
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        return True
    
    def sort_object(self, obj):
        """
        Sort a single object: pick it up and place in appropriate bin.
        """
        color = obj.color.lower()
        
        if color not in self.BIN_LOCATIONS:
            rospy.logwarn(f"Unknown color: {color}, skipping object")
            return False
        
        rospy.loginfo(f"Sorting {color} object...")
        
        # Pick up the object
        pick_z = obj.position.z - 0.7  # Adjust for table height relative to robot base
        if not self.pick_object(obj.position.x, obj.position.y, pick_z):
            rospy.logerr("Pick operation failed")
            return False
        
        # Get bin location
        bin_x, bin_y, bin_z = self.BIN_LOCATIONS[color]
        
        # Place in bin
        if not self.place_object(bin_x, bin_y, bin_z):
            rospy.logerr("Place operation failed")
            return False
        
        rospy.loginfo(f"Successfully sorted {color} object!")
        return True
    
    def control_loop(self, event):
        """Main control loop - called periodically."""
        if self.is_busy:
            return
        
        if not self.detected_objects:
            return
        
        # Get the first detected object
        obj = self.detected_objects[0]
        
        # Mark as busy
        self.is_busy = True
        
        try:
            # Sort the object
            success = self.sort_object(obj)
            
            if success:
                # Remove sorted object from list
                self.detected_objects = self.detected_objects[1:]
            
            # Return to home position
            self.move_to_home()
            
        except Exception as e:
            rospy.logerr(f"Error in sorting: {e}")
            self.move_to_home()
        
        finally:
            self.is_busy = False
    
    def run(self):
        """Run the controller."""
        rospy.spin()


def main():
    try:
        controller = SortingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
