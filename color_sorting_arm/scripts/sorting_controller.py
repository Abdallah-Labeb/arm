#!/usr/bin/env python3
"""
Sorting Controller Node
Main controller that orchestrates the complete sorting pipeline:
1. Detects colored cubes
2. Estimates their 3D positions
3. Plans pick-and-place motions
4. Controls gripper
5. Sorts cubes into designated zones
"""

import rospy
from std_msgs.msg import Bool
from color_sorting_arm.msg import Object3DArray
from motion_planner import MotionPlanner
import time


class SortingController:
    def __init__(self):
        rospy.init_node('sorting_controller', anonymous=True)
        
        # Motion planner instance
        self.motion_planner = MotionPlanner()
        
        # Gripper command publisher
        self.gripper_pub = rospy.Publisher('/gripper_command', Bool, queue_size=10)
        
        # Detected objects
        self.detected_objects = []
        self.processing = False
        
        # Sorting zone positions (x, y, z)
        self.sorting_zones = {
            'red': (0.2, 0.3, 0.85),
            'blue': (0.2, 0.0, 0.85),
            'green': (0.2, -0.3, 0.85)
        }
        
        # Subscribe to object positions
        self.object_sub = rospy.Subscriber('/object_positions', Object3DArray, 
                                          self.object_callback)
        
        rospy.loginfo("Sorting Controller Node initialized")
        rospy.loginfo("Waiting for objects to detect...")
    
    def object_callback(self, msg):
        """Store detected objects"""
        if not self.processing and len(msg.objects) > 0:
            self.detected_objects = msg.objects
            rospy.loginfo(f"Detected {len(self.detected_objects)} objects")
    
    def open_gripper(self):
        """Send command to open gripper"""
        self.gripper_pub.publish(True)
        rospy.sleep(1.5)
    
    def close_gripper(self):
        """Send command to close gripper"""
        self.gripper_pub.publish(False)
        rospy.sleep(1.5)
    
    def sort_single_object(self, obj):
        """Sort a single object"""
        color = obj.color
        pos = obj.position
        
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"Sorting {color} cube")
        rospy.loginfo(f"Current position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        
        # Open gripper
        self.open_gripper()
        
        # Pick object
        rospy.loginfo(f"Step 1: Moving to pick {color} cube...")
        self.motion_planner.pick_object(pos.x, pos.y, pos.z)
        
        # Close gripper to grab
        rospy.loginfo(f"Step 2: Grabbing {color} cube...")
        self.close_gripper()
        
        # Get sorting zone for this color
        if color in self.sorting_zones:
            zone_x, zone_y, zone_z = self.sorting_zones[color]
            rospy.loginfo(f"Step 3: Moving to {color} zone ({zone_x}, {zone_y}, {zone_z})...")
            
            # Place object
            self.motion_planner.place_object(zone_x, zone_y, zone_z)
            
            # Open gripper to release
            rospy.loginfo(f"Step 4: Releasing {color} cube...")
            self.open_gripper()
            
            rospy.loginfo(f"Successfully sorted {color} cube!")
        else:
            rospy.logwarn(f"Unknown color: {color}")
            self.open_gripper()
        
        # Return to home
        rospy.loginfo("Returning to home position...")
        self.motion_planner.move_to_home()
        
        rospy.loginfo(f"{'='*50}\n")
    
    def sort_all_objects(self):
        """Sort all detected objects"""
        if len(self.detected_objects) == 0:
            rospy.loginfo("No objects to sort")
            return
        
        self.processing = True
        
        rospy.loginfo(f"\n{'#'*50}")
        rospy.loginfo(f"# Starting sorting sequence")
        rospy.loginfo(f"# Total objects: {len(self.detected_objects)}")
        rospy.loginfo(f"{'#'*50}\n")
        
        # Sort objects by distance (closest first)
        sorted_objects = sorted(self.detected_objects, 
                              key=lambda obj: obj.position.x**2 + obj.position.y**2)
        
        # Process each object
        for i, obj in enumerate(sorted_objects):
            rospy.loginfo(f"\nProcessing object {i+1}/{len(sorted_objects)}")
            self.sort_single_object(obj)
            rospy.sleep(1.0)
        
        rospy.loginfo(f"\n{'#'*50}")
        rospy.loginfo(f"# Sorting sequence completed!")
        rospy.loginfo(f"# All {len(sorted_objects)} objects sorted successfully")
        rospy.loginfo(f"{'#'*50}\n")
        
        self.detected_objects = []
        self.processing = False
    
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(0.5)  # 0.5 Hz (check every 2 seconds)
        
        rospy.loginfo("Sorting controller ready. Waiting for objects...")
        
        # Wait a bit for everything to initialize
        rospy.sleep(3.0)
        
        while not rospy.is_shutdown():
            if not self.processing and len(self.detected_objects) > 0:
                # Start sorting
                rospy.loginfo("\nObjects detected! Starting sorting sequence in 2 seconds...")
                rospy.sleep(2.0)
                self.sort_all_objects()
            
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = SortingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
