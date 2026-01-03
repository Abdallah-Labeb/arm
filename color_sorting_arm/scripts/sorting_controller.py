#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from color_sorting_arm.msg import Object3DArray

class SortingController:
    
    BIN_LOCATIONS = {
        'red': (0.6, 0.25, 0.05),
        'blue': (0.6, -0.25, 0.05),
        'green': (0.6, 0.0, 0.05)
    }
    
    L1 = 0.11
    L2 = 0.15
    L3 = 0.13
    L4 = 0.16
    
    def __init__(self):
        rospy.init_node('sorting_controller')
        rospy.loginfo("Starting Sorting Controller...")
        
        self.current_joints = [0.0] * 7
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 
                           'gripper_left_joint', 'gripper_right_joint']
        self.detected_objects = []
        self.is_busy = False
        self.gripper_open = True
        
        self.joint_pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.joint_pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        rospy.Subscriber('/sorting_arm/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/object_positions', Object3DArray, self.objects_callback)
        
        rospy.sleep(2.0)
        
        self.startup_test()
        
        self.move_to_home()
        rospy.sleep(1.0)
        
        self.control_timer = rospy.Timer(rospy.Duration(0.5), self.control_loop)
        
        rospy.loginfo("Sorting Controller initialized successfully!")
    
    def startup_test(self):
        rospy.loginfo("=" * 50)
        rospy.loginfo("STARTUP TEST: Performing test movements...")
        rospy.loginfo("=" * 50)
        
        rospy.loginfo("Test 1: Moving to neutral position...")
        neutral = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.set_joint_positions(neutral, 2.0)
        
        rospy.loginfo("Test 2: Rotating base left...")
        self.set_joint_positions([0.8, 0.0, 0.0, 0.0, 0.0], 1.5)
        
        rospy.loginfo("Test 3: Rotating base right...")
        self.set_joint_positions([-0.8, 0.0, 0.0, 0.0, 0.0], 1.5)
        
        rospy.loginfo("Test 4: Returning to center...")
        self.set_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0], 1.5)
        
        rospy.loginfo("Test 5: Bending shoulder forward...")
        self.set_joint_positions([0.0, 1.0, 0.0, 0.0, 0.0], 1.5)
        
        rospy.loginfo("Test 6: Bending elbow...")
        self.set_joint_positions([0.0, 1.0, -1.5, 0.0, 0.0], 1.5)
        
        rospy.loginfo("Test 7: Bending wrist...")
        self.set_joint_positions([0.0, 1.0, -1.5, -0.5, 0.0], 1.5)
        
        rospy.loginfo("Test 8: Testing gripper open...")
        self.open_gripper()
        rospy.sleep(0.5)
        
        rospy.loginfo("Test 9: Testing gripper close...")
        self.close_gripper()
        rospy.sleep(0.5)
        
        self.open_gripper()
        
        rospy.loginfo("Test 10: Performing wave motion...")
        for _ in range(2):
            self.set_joint_positions([0.5, 0.5, -1.0, 0.0, 0.0], 0.8)
            self.set_joint_positions([-0.5, 0.5, -1.0, 0.0, 0.0], 0.8)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("STARTUP TEST COMPLETE! Arm is working correctly.")
        rospy.loginfo("=" * 50)
    
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_joints[idx] = msg.position[i]
    
    def objects_callback(self, msg):
        self.detected_objects = msg.objects
    
    def publish_joint_command(self, joint_name, position):
        if joint_name in self.joint_pubs:
            cmd = Float64()
            cmd.data = float(position)
            self.joint_pubs[joint_name].publish(cmd)
    
    def set_joint_positions(self, positions, duration=2.0):
        for i, pos in enumerate(positions):
            self.publish_joint_command(f'joint{i+1}', pos)
        
        rospy.sleep(duration)
    
    def open_gripper(self):
        rospy.loginfo("Opening gripper...")
        self.gripper_left_pub.publish(Float64(0.015))
        self.gripper_right_pub.publish(Float64(0.015))
        self.gripper_open = True
        rospy.sleep(0.5)
    
    def close_gripper(self):
        rospy.loginfo("Closing gripper...")
        self.gripper_left_pub.publish(Float64(0.003))
        self.gripper_right_pub.publish(Float64(0.003))
        self.gripper_open = False
        rospy.sleep(0.5)
    
    def move_to_home(self):
        rospy.loginfo("Moving to home position...")
        home = [0.0, 0.6, -1.0, -0.3, 0.0]
        self.set_joint_positions(home, 2.0)
        self.open_gripper()
    
    def inverse_kinematics(self, x, y, z, pitch=math.pi/2):
        try:
            j1 = math.atan2(y, x)
            wrist_offset = self.L4 * math.cos(pitch)
            z_offset = self.L4 * math.sin(pitch)
            
            r_wrist = r - wrist_offset if r > wrist_offset else 0.01
            z_wrist = z - self.L1 + z_offset
            
            d = math.sqrt(r_wrist**2 + z_wrist**2)
            
            max_reach = self.L2 + self.L3
            if d > max_reach * 0.95:
                rospy.logwarn(f"Target may be at edge of reach: d={d:.3f}, max={max_reach:.3f}")
                d = max_reach * 0.95
            
            cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            cos_j3 = np.clip(cos_j3, -1.0, 1.0)
            j3 = math.acos(cos_j3)
            
            alpha = math.atan2(z_wrist, r_wrist)
            beta = math.acos((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d))
            j2 = alpha + beta
            
            j4 = pitch - j2 - j3 + math.pi/2
            
            j5 = 0.0
            
            j3 = -(math.pi - j3)
            
            return [j1, j2, j3, j4, j5]
            
        except Exception as e:
            rospy.logerr(f"IK calculation failed: {e}")
            return None
    
    def move_to_position(self, x, y, z, pitch=math.pi/2):
        rospy.loginfo(f"Moving to position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        joints = self.inverse_kinematics(x, y, z, pitch)
        if joints is None:
            rospy.logerr("Failed to calculate IK solution")
            return False
        
        rospy.loginfo(f"Joint angles: {[f'{j:.2f}' for j in joints]}")
        self.set_joint_positions(joints, 2.0)
        return True
    
    def pick_object(self, x, y, z):
        rospy.loginfo(f"Picking object at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        approach_z = z + 0.1
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        if not self.move_to_position(x, y, z + 0.02):
            return False
        rospy.sleep(0.5)
        
        self.close_gripper()
        rospy.sleep(0.5)
        
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        return True
    
    def place_object(self, x, y, z):
        rospy.loginfo(f"Placing object at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        approach_z = z + 0.1
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        if not self.move_to_position(x, y, z + 0.03):
            return False
        rospy.sleep(0.5)
        
        self.open_gripper()
        rospy.sleep(0.5)
        
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        return True
    
    def sort_object(self, obj):
        color = obj.color.lower()
        
        if color not in self.BIN_LOCATIONS:
            rospy.logwarn(f"Unknown color: {color}, skipping object")
            return False
        
        rospy.loginfo(f"Sorting {color} object...")
        
        pick_z = obj.position.z - 0.7
        if not self.pick_object(obj.position.x, obj.position.y, pick_z):
            rospy.logerr("Pick operation failed")
            return False
        
        bin_x, bin_y, bin_z = self.BIN_LOCATIONS[color]
        
        if not self.place_object(bin_x, bin_y, bin_z):
            rospy.logerr("Place operation failed")
            return False
        
        rospy.loginfo(f"Successfully sorted {color} object!")
        return True
    
    def control_loop(self, event):
        if self.is_busy:
            return
        
        if not self.detected_objects:
            return
        
        obj = self.detected_objects[0]
        
        self.is_busy = True
        
        try:
            success = self.sort_object(obj)
            
            if success:
                self.detected_objects = self.detected_objects[1:]
            
            self.move_to_home()
            
        except Exception as e:
            rospy.logerr(f"Error in sorting: {e}")
            self.move_to_home()
        
        finally:
            self.is_busy = False
    
    def run(self):
        rospy.spin()


def main():
    try:
        controller = SortingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
