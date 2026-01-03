#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


class MotionPlanner:
    def __init__(self):
        rospy.init_node('motion_planner', anonymous=True)
        
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
        
        self.current_joints = None
        
        self.joint_sub = rospy.Subscriber('/sorting_arm/joint_states', JointState, 
                                         self.joint_state_callback)
        
        self.L1 = 0.2
        self.L2 = 0.3
        self.L3 = 0.3
        self.L4 = 0.2
        self.L5 = 0.1
        
        rospy.loginfo("Motion Planner Node initialized")
        
        rospy.sleep(1.0)
        
        self.move_to_home()
    
    def joint_state_callback(self, msg):
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if 'joint' in name and 'gripper' not in name:
                self.current_joints[name] = msg.position[i]
    
    def move_to_home(self):
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
        for joint_name, position in joint_positions.items():
            if joint_name in self.joint_pubs:
                self.joint_pubs[joint_name].publish(position)
        
        rospy.sleep(duration)
    
    def inverse_kinematics(self, target_x, target_y, target_z):
        j1 = np.arctan2(target_y, target_x)
        
        r = np.sqrt(target_x**2 + target_y**2)
        
        z = target_z - self.L1
        
        gripper_offset = self.L4 + self.L5
        
        r_wrist = r - gripper_offset * 0.3
        z_wrist = z + gripper_offset * 0.3
        
        d = np.sqrt(r_wrist**2 + z_wrist**2)
        
        if d > (self.L2 + self.L3):
            rospy.logwarn(f"Target may be out of reach: d={d:.2f}, max={self.L2 + self.L3:.2f}")
            d = self.L2 + self.L3 - 0.01
        
        cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_j3 = np.clip(cos_j3, -1, 1)
        j3 = np.arccos(cos_j3)
        
        alpha = np.arctan2(z_wrist, r_wrist)
        beta = np.arctan2(self.L3 * np.sin(j3), self.L2 + self.L3 * np.cos(j3))
        j2 = alpha - beta
        
        j4 = -(j2 + j3) - np.pi/4
        
        j5 = -j1
        
        j1 = np.clip(j1, -np.pi, np.pi)
        j2 = np.clip(j2, -np.pi/2, np.pi/2)
        j3 = np.clip(j3, -np.pi/2, np.pi/2)
        j4 = np.clip(j4, -np.pi/2, np.pi/2)
        j5 = np.clip(j5, -np.pi, np.pi)
        
        return [j1, j2, j3, j4, j5]
    
    def move_to_position(self, x, y, z):
        rospy.loginfo(f"Moving to position: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        try:
            joint_angles = self.inverse_kinematics(x, y, z)
            
            joint_positions = {
                'joint1': joint_angles[0],
                'joint2': joint_angles[1],
                'joint3': joint_angles[2],
                'joint4': joint_angles[3],
                'joint5': joint_angles[4],
            }
            
            self.move_joints(joint_positions, duration=3.0)
            
            rospy.loginfo("Reached target position")
            return True
            
        except Exception as e:
            rospy.logerr(f"IK Error: {e}")
            return False
    
    def pick_object(self, x, y, z):
        rospy.loginfo(f"Picking object at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        above_z = z + 0.15
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
        
        self.move_to_position(x, y, z)
        rospy.sleep(0.5)
        
        rospy.sleep(1.0)
        
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
    
    def place_object(self, x, y, z):
        rospy.loginfo(f"Placing object at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        above_z = z + 0.15
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
        
        self.move_to_position(x, y, z + 0.05)
        rospy.sleep(0.5)
        
        rospy.sleep(1.0)
        
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = MotionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
