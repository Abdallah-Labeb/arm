#!/usr/bin/env python3
"""
Motion Planner Node
Plans and executes arm movements for pick-and-place operations
Uses simple inverse kinematics for 5-DOF arm
"""

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


class MotionPlanner:
    def __init__(self):
        rospy.init_node('motion_planner', anonymous=True)
        
        # Publishers for joint controllers
        self.joint_pubs = {
            'joint1': rospy.Publisher('/sorting_arm/joint1_position_controller/command', 
                                     Float64, queue_size=10),
            'joint2': rospy.Publisher('/sorting_arm/joint2_position_controller/command', 
                                     Float64, queue_size=10),
            'joint3': rospy.Publisher('/sorting_arm/joint3_position_controller/command', 
                                     Float64, queue_size=10),
            'joint4': rospy.Publisher('/sorting_arm/joint4_position_controller/command', 
                                     Float64, queue_size=10),
            'joint5': rospy.Publisher('/sorting_arm/joint5_position_controller/command', 
                                     Float64, queue_size=10),
        }
        
        # Current joint states
        self.current_joints = None
        
        # Subscriber for joint states
        self.joint_sub = rospy.Subscriber('/sorting_arm/joint_states', JointState, 
                                         self.joint_state_callback)
        
        # Link lengths (from URDF)
        self.L1 = 0.2   # Link 1 height
        self.L2 = 0.3   # Link 2 length
        self.L3 = 0.3   # Link 3 length
        self.L4 = 0.2   # Link 4 length
        self.L5 = 0.1   # Link 5 to gripper
        
        rospy.loginfo("Motion Planner Node initialized")
        
        # Wait for joint states
        rospy.sleep(1.0)
        
        # Move to home position
        self.move_to_home()
    
    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if 'joint' in name and 'gripper' not in name:
                self.current_joints[name] = msg.position[i]
    
    def move_to_home(self):
        """Move arm to home position"""
        rospy.loginfo("Moving to home position")
        home_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
        }
        self.move_joints(home_positions)
    
    def move_joints(self, joint_positions, duration=2.0):
        """
        Move joints to specified positions
        joint_positions: dict of {joint_name: position}
        """
        # Publish joint commands
        for joint_name, position in joint_positions.items():
            if joint_name in self.joint_pubs:
                self.joint_pubs[joint_name].publish(position)
        
        rospy.sleep(duration)
    
    def inverse_kinematics(self, target_x, target_y, target_z):
        """
        Simple inverse kinematics for 5-DOF arm
        Returns joint angles [j1, j2, j3, j4, j5]
        """
        # Joint 1 (base rotation) - rotate to face target
        j1 = np.arctan2(target_y, target_x)
        
        # Horizontal reach distance
        r = np.sqrt(target_x**2 + target_y**2)
        
        # Vertical height from joint 2
        z = target_z - self.L1
        
        # Account for end effector length
        gripper_offset = self.L4 + self.L5
        
        # Target for wrist (before end effector)
        r_wrist = r - gripper_offset * 0.3  # Approximate horizontal offset
        z_wrist = z + gripper_offset * 0.3  # Approximate vertical offset
        
        # Distance to wrist target
        d = np.sqrt(r_wrist**2 + z_wrist**2)
        
        # Check if target is reachable
        if d > (self.L2 + self.L3):
            rospy.logwarn(f"Target may be out of reach: d={d:.2f}, max={self.L2 + self.L3:.2f}")
            d = self.L2 + self.L3 - 0.01
        
        # Joint 3 (elbow) - using cosine law
        cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_j3 = np.clip(cos_j3, -1, 1)
        j3 = np.arccos(cos_j3)
        
        # Joint 2 (shoulder)
        alpha = np.arctan2(z_wrist, r_wrist)
        beta = np.arctan2(self.L3 * np.sin(j3), self.L2 + self.L3 * np.cos(j3))
        j2 = alpha - beta
        
        # Joint 4 (wrist pitch) - point downward
        j4 = -(j2 + j3) - np.pi/4
        
        # Joint 5 (wrist roll) - align with base
        j5 = -j1
        
        # Apply joint limits
        j1 = np.clip(j1, -np.pi, np.pi)
        j2 = np.clip(j2, -np.pi/2, np.pi/2)
        j3 = np.clip(j3, -np.pi/2, np.pi/2)
        j4 = np.clip(j4, -np.pi/2, np.pi/2)
        j5 = np.clip(j5, -np.pi, np.pi)
        
        return [j1, j2, j3, j4, j5]
    
    def move_to_position(self, x, y, z):
        """Move end effector to target position"""
        rospy.loginfo(f"Moving to position: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        try:
            # Calculate inverse kinematics
            joint_angles = self.inverse_kinematics(x, y, z)
            
            # Create joint position dict
            joint_positions = {
                'joint1': joint_angles[0],
                'joint2': joint_angles[1],
                'joint3': joint_angles[2],
                'joint4': joint_angles[3],
                'joint5': joint_angles[4],
            }
            
            # Move to position
            self.move_joints(joint_positions, duration=3.0)
            
            rospy.loginfo("Reached target position")
            return True
            
        except Exception as e:
            rospy.logerr(f"IK Error: {e}")
            return False
    
    def pick_object(self, x, y, z):
        """Execute pick sequence"""
        rospy.loginfo(f"Picking object at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Move above object
        above_z = z + 0.15
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
        
        # Move down to object
        self.move_to_position(x, y, z)
        rospy.sleep(0.5)
        
        # Close gripper (will be handled by gripper controller)
        rospy.sleep(1.0)
        
        # Lift object
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
    
    def place_object(self, x, y, z):
        """Execute place sequence"""
        rospy.loginfo(f"Placing object at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Move above placement location
        above_z = z + 0.15
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
        
        # Move down to placement height
        self.move_to_position(x, y, z + 0.05)
        rospy.sleep(0.5)
        
        # Open gripper (will be handled by gripper controller)
        rospy.sleep(1.0)
        
        # Move up
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = MotionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
