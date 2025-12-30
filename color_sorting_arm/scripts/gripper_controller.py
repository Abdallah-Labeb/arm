#!/usr/bin/env python3
"""
Gripper Controller Node
Controls the gripper to open and close for picking/placing cubes
"""

import rospy
from std_msgs.msg import Float64, Bool


class GripperController:
    def __init__(self):
        rospy.init_node('gripper_controller', anonymous=True)
        
        # Publishers for gripper joints
        self.left_pub = rospy.Publisher('/sorting_arm/gripper_left_position_controller/command', 
                                        Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/sorting_arm/gripper_right_position_controller/command', 
                                         Float64, queue_size=10)
        
        # Subscriber for gripper commands
        self.command_sub = rospy.Subscriber('/gripper_command', Bool, self.command_callback)
        
        # Gripper state
        self.is_open = True
        
        rospy.loginfo("Gripper Controller Node initialized")
        
        # Start with open gripper
        self.open_gripper()
    
    def open_gripper(self):
        """Open the gripper"""
        rospy.loginfo("Opening gripper")
        self.left_pub.publish(0.03)   # Left finger moves positive
        self.right_pub.publish(-0.03) # Right finger moves negative
        self.is_open = True
        rospy.sleep(1.0)  # Wait for gripper to open
    
    def close_gripper(self):
        """Close the gripper"""
        rospy.loginfo("Closing gripper")
        self.left_pub.publish(0.0)
        self.right_pub.publish(0.0)
        self.is_open = False
        rospy.sleep(1.0)  # Wait for gripper to close
    
    def command_callback(self, msg):
        """
        Handle gripper command
        True = Open, False = Close
        """
        if msg.data and not self.is_open:
            self.open_gripper()
        elif not msg.data and self.is_open:
            self.close_gripper()
    
    def run(self):
        """Keep the node running"""
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = GripperController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
