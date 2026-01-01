#!/usr/bin/env python3
"""
Gripper Controller Node
Controls the gripper to open and close for picking/placing cubes
"""

import rospy
from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool, SetBoolResponse


class GripperController:
    """Controls the parallel gripper for grasping objects."""
    
    # Gripper parameters (from URDF)
    GRIPPER_OPEN = 0.02    # Maximum opening
    GRIPPER_CLOSED = 0.005  # Closed position (small gap for gripping)
    
    def __init__(self):
        rospy.init_node('gripper_controller')
        rospy.loginfo("Starting Gripper Controller...")
        
        # Publishers for gripper joints
        self.left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', 
            Float64, queue_size=10)
        self.right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', 
            Float64, queue_size=10)
        
        # Subscriber for gripper commands
        self.command_sub = rospy.Subscriber('/gripper_command', Bool, self.command_callback)
        
        # Service for gripper control
        self.gripper_service = rospy.Service('/gripper_control', SetBool, self.service_callback)
        
        # Gripper state
        self.is_open = True
        
        rospy.loginfo("Gripper Controller initialized")
        
        # Wait for publishers to connect
        rospy.sleep(0.5)
        
        # Start with open gripper
        self.open_gripper()
    
    def set_gripper_position(self, position):
        """
        Set gripper finger positions.
        Both fingers move symmetrically.
        """
        cmd = Float64()
        cmd.data = position
        self.left_pub.publish(cmd)
        self.right_pub.publish(cmd)
    
    def open_gripper(self):
        """Open the gripper fully."""
        rospy.loginfo("Opening gripper")
        self.set_gripper_position(self.GRIPPER_OPEN)
        self.is_open = True
        rospy.sleep(0.5)
    
    def close_gripper(self):
        """Close the gripper to grasp an object."""
        rospy.loginfo("Closing gripper")
        self.set_gripper_position(self.GRIPPER_CLOSED)
        self.is_open = False
        rospy.sleep(0.5)
    
    def command_callback(self, msg):
        """
        Handle gripper command from topic.
        True = Open, False = Close
        """
        if msg.data and not self.is_open:
            self.open_gripper()
        elif not msg.data and self.is_open:
            self.close_gripper()
    
    def service_callback(self, req):
        """
        Handle gripper command from service.
        True = Open, False = Close
        """
        if req.data:
            self.open_gripper()
            return SetBoolResponse(success=True, message="Gripper opened")
        else:
            self.close_gripper()
            return SetBoolResponse(success=True, message="Gripper closed")
    
    def run(self):
        """Keep the node running."""
        rospy.spin()


def main():
    try:
        controller = GripperController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
