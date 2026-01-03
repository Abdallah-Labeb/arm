#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool, SetBoolResponse


class GripperController:
    
    GRIPPER_OPEN = 0.02
    GRIPPER_CLOSED = 0.005
    
    def __init__(self):
        rospy.init_node('gripper_controller')
        rospy.loginfo("Starting Gripper Controller...")
        
        self.left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', 
            Float64, queue_size=10)
        self.right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', 
            Float64, queue_size=10)
        
        self.command_sub = rospy.Subscriber('/gripper_command', Bool, self.command_callback)
        
        self.gripper_service = rospy.Service('/gripper_control', SetBool, self.service_callback)
        
        self.is_open = True
        
        rospy.loginfo("Gripper Controller initialized")
        
        rospy.sleep(0.5)
        
        self.open_gripper()
    
    def set_gripper_position(self, position):
        cmd = Float64()
        cmd.data = position
        self.left_pub.publish(cmd)
        self.right_pub.publish(cmd)
    
    def open_gripper(self):
        rospy.loginfo("Opening gripper")
        self.set_gripper_position(self.GRIPPER_OPEN)
        self.is_open = True
        rospy.sleep(0.5)
    
    def close_gripper(self):
        rospy.loginfo("Closing gripper")
        self.set_gripper_position(self.GRIPPER_CLOSED)
        self.is_open = False
        rospy.sleep(0.5)
    
    def command_callback(self, msg):
        if msg.data and not self.is_open:
            self.open_gripper()
        elif not msg.data and self.is_open:
            self.close_gripper()
    
    def service_callback(self, req):
        if req.data:
            self.open_gripper()
            return SetBoolResponse(success=True, message="Gripper opened")
        else:
            self.close_gripper()
            return SetBoolResponse(success=True, message="Gripper closed")
    
    def run(self):
        rospy.spin()


def main():
    try:
        controller = GripperController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
