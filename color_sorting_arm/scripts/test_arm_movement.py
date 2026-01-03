#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import sys

class ArmTester:
    def __init__(self):
        rospy.init_node('arm_startup_test', anonymous=True)
        
        self.pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        rospy.loginfo("Waiting for Gazebo and controllers...")
        rospy.sleep(3.0)

    def move(self, j1, j2, j3, j4, j5, duration=1.5):
        self.pubs['joint1'].publish(Float64(j1))
        self.pubs['joint2'].publish(Float64(j2))
        self.pubs['joint3'].publish(Float64(j3))
        self.pubs['joint4'].publish(Float64(j4))
        self.pubs['joint5'].publish(Float64(j5))
        rospy.sleep(duration)

    def gripper(self, open_close):
        if open_close == "open":
            self.gripper_left.publish(Float64(0.02))
            self.gripper_right.publish(Float64(0.02))
        else:
            self.gripper_left.publish(Float64(0.0))
            self.gripper_right.publish(Float64(0.0))
        rospy.sleep(0.5)

    def run_test(self):
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("    ARM STARTUP TEST - Verifying arm operation")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        
        rospy.loginfo("[1/10] Moving to start position (standing up)...")
        self.move(0, 0, 0, 0, 0, 2.0)
        
        rospy.loginfo("[2/10] Rotating base LEFT...")
        self.move(1.0, 0, 0, 0, 0, 1.5)
        
        rospy.loginfo("[3/10] Rotating base RIGHT...")
        self.move(-1.0, 0, 0, 0, 0, 1.5)
        
        rospy.loginfo("[4/10] Centering base...")
        self.move(0, 0, 0, 0, 0, 1.0)
        
        rospy.loginfo("[5/10] Bending shoulder FORWARD...")
        self.move(0, 1.2, 0, 0, 0, 1.5)
        
        rospy.loginfo("[6/10] Bending elbow...")
        self.move(0, 1.2, -1.5, 0, 0, 1.5)
        
        rospy.loginfo("[7/10] Bending wrist DOWN...")
        self.move(0, 1.2, -1.5, -0.8, 0, 1.5)
        
        rospy.loginfo("[8/10] Testing gripper - OPEN...")
        self.gripper("open")
        rospy.sleep(0.5)
        
        rospy.loginfo("[9/10] Testing gripper - CLOSE...")
        self.gripper("close")
        rospy.sleep(0.5)
        self.gripper("open")
        
        rospy.loginfo("[10/10] Moving to READY position...")
        self.move(0, 0.8, -1.2, -0.4, 0, 2.0)
        
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("    TEST COMPLETE! Arm is working correctly!")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")


def main():
    try:
        tester = ArmTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
