#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
from color_sorting_arm.msg import DetectedObject, DetectedObjectArray


class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        self.color_ranges = {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'blue': [
                (np.array([100, 100, 100]), np.array([130, 255, 255]))
            ],
            'green': [
                (np.array([40, 100, 100]), np.array([80, 255, 255]))
            ]
        }
        
        self.detection_pub = rospy.Publisher('/detected_objects', DetectedObjectArray, queue_size=10)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)
        
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("Color Detector Node initialized")
    
    def detect_color(self, hsv_image, color_name):
        mask = None
        
        for lower, upper in self.color_ranges[color_name]:
            if mask is None:
                mask = cv2.inRange(hsv_image, lower, upper)
            else:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
    
    def find_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centers = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy, area))
        
        return centers, contours
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        detected_objects = DetectedObjectArray()
        detected_objects.header = msg.header
        
        for color_name in ['red', 'blue', 'green']:
            mask = self.detect_color(hsv, color_name)
            centers, contours = self.find_contours(mask)
            
            for cx, cy, area in centers:
                obj = DetectedObject()
                obj.color = color_name
                obj.pixel_x = cx
                obj.pixel_y = cy
                obj.area = area
                detected_objects.objects.append(obj)
                
                color_bgr = {'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0)}
                cv2.circle(cv_image, (cx, cy), 5, color_bgr[color_name], -1)
                cv2.putText(cv_image, color_name, (cx + 10, cy), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr[color_name], 2)
            
            cv2.drawContours(cv_image, contours, -1, (0, 255, 255), 2)
        
        if len(detected_objects.objects) > 0:
            self.detection_pub.publish(detected_objects)
        
        try:
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        detector = ColorDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
