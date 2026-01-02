#!/usr/bin/env python3
"""
Camera Rotation Test Script
Tests different camera orientations to find the correct one that sees the table.
Rotates camera through different angles and analyzes what it sees.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft
import time


class CameraRotationTest:
    def __init__(self):
        rospy.init_node('camera_rotation_test', anonymous=True)
        
        self.bridge = CvBridge()
        self.current_image = None
        self.image_received = False
        
        # Subscribe to camera
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Wait for Gazebo service
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("CAMERA ROTATION TEST - Finding correct orientation")
        rospy.loginfo("=" * 70)
        rospy.loginfo("Camera body = BLUE box, Lens = RED cylinder")
        rospy.loginfo("Lens points in direction camera is looking")
        rospy.loginfo("=" * 70)
        
    def image_callback(self, msg):
        """Receive camera images."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
    
    def set_camera_pose(self, x, y, z, roll, pitch, yaw):
        """Set camera model pose in Gazebo."""
        model_state = ModelState()
        model_state.model_name = 'overhead_camera'
        
        # Set position
        model_state.pose.position = Point(x, y, z)
        
        # Convert RPY to quaternion
        quat = tft.quaternion_from_euler(roll, pitch, yaw)
        model_state.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        
        model_state.reference_frame = 'world'
        
        try:
            resp = self.set_model_state(model_state)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def analyze_image(self):
        """Analyze current image quality."""
        if self.current_image is None:
            return None
        
        img = self.current_image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Calculate statistics
        mean_val = img.mean()
        std_val = img.std()
        min_val = img.min()
        max_val = img.max()
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        edge_count = np.count_nonzero(edges)
        
        # Color variation
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h_range = hsv[:,:,0].max() - hsv[:,:,0].min()
        s_range = hsv[:,:,1].max() - hsv[:,:,1].min()
        v_range = hsv[:,:,2].max() - hsv[:,:,2].min()
        
        # Score: higher is better
        # Good image has: high std, many edges, color variation
        score = std_val * 0.5 + edge_count * 0.001 + (h_range + s_range + v_range) * 0.1
        
        return {
            'mean': mean_val,
            'std': std_val,
            'min': min_val,
            'max': max_val,
            'edges': edge_count,
            'h_range': h_range,
            's_range': s_range,
            'v_range': v_range,
            'score': score
        }
    
    def test_orientations(self):
        """Test multiple camera orientations."""
        rospy.sleep(2.0)  # Wait for startup
        
        # Camera position above table
        cam_x, cam_y, cam_z = 0.35, 0.0, 1.0
        
        # Test different orientations
        # Format: (name, roll, pitch, yaw)
        test_cases = [
            ("Default (lens forward +X)", 0, 0, 0),
            ("Pitch down 90° (lens points -Z)", 0, 1.5708, 0),
            ("Pitch down 90° + Roll 180°", 3.14159, 1.5708, 0),
            ("Pitch up 90° (lens points +Z)", 0, -1.5708, 0),
            ("Yaw left 90°, Pitch down 90°", 0, 1.5708, 1.5708),
            ("Yaw right 90°, Pitch down 90°", 0, 1.5708, -1.5708),
            ("Roll 90°, Pitch down 90°", 1.5708, 1.5708, 0),
            ("Roll -90°, Pitch down 90°", -1.5708, 1.5708, 0),
            ("Pitch 45° down", 0, 0.785, 0),
            ("Pitch 135° (lens points back+down)", 0, 2.356, 0),
            ("Yaw 180°, Pitch down 90°", 0, 1.5708, 3.14159),
            ("Roll 180° only", 3.14159, 0, 0),
        ]
        
        results = []
        
        for i, (name, roll, pitch, yaw) in enumerate(test_cases):
            rospy.loginfo("")
            rospy.loginfo("=" * 70)
            rospy.loginfo(f"TEST {i+1}/{len(test_cases)}: {name}")
            rospy.loginfo(f"  Roll={roll:.4f}, Pitch={pitch:.4f}, Yaw={yaw:.4f}")
            rospy.loginfo("=" * 70)
            
            # Set camera pose
            success = self.set_camera_pose(cam_x, cam_y, cam_z, roll, pitch, yaw)
            if not success:
                rospy.logwarn(f"  Failed to set pose!")
                continue
            
            # Wait for image to update
            rospy.sleep(1.5)
            
            # Analyze image
            analysis = self.analyze_image()
            if analysis is None:
                rospy.logwarn("  No image received!")
                continue
            
            # Print results
            rospy.loginfo(f"  Image Stats:")
            rospy.loginfo(f"    Mean: {analysis['mean']:.2f}")
            rospy.loginfo(f"    Std Dev: {analysis['std']:.2f}")
            rospy.loginfo(f"    Range: [{analysis['min']}, {analysis['max']}]")
            rospy.loginfo(f"    Edges: {analysis['edges']} pixels")
            rospy.loginfo(f"    HSV Ranges: H={analysis['h_range']}, S={analysis['s_range']}, V={analysis['v_range']}")
            rospy.loginfo(f"  SCORE: {analysis['score']:.2f}")
            
            # Save image
            filename = f"/tmp/camera_test_angle_{i+1:02d}.jpg"
            cv2.imwrite(filename, self.current_image)
            rospy.loginfo(f"  Saved: {filename}")
            
            # Determine if good
            if analysis['score'] > 50:
                rospy.loginfo(f"  ✓ EXCELLENT - This orientation sees the scene!")
            elif analysis['score'] > 20:
                rospy.loginfo(f"  ✓ GOOD - Some scene details visible")
            elif analysis['std'] < 5:
                rospy.logwarn(f"  ✗ BAD - Uniform color (likely sky/ground)")
            else:
                rospy.loginfo(f"  ~ FAIR - Limited scene details")
            
            results.append((name, roll, pitch, yaw, analysis['score']))
        
        # Print summary
        rospy.loginfo("")
        rospy.loginfo("=" * 70)
        rospy.loginfo("FINAL RESULTS - Sorted by Score")
        rospy.loginfo("=" * 70)
        
        results.sort(key=lambda x: x[4], reverse=True)
        
        for i, (name, roll, pitch, yaw, score) in enumerate(results):
            rospy.loginfo(f"{i+1}. {name}")
            rospy.loginfo(f"   Roll={roll:.4f}, Pitch={pitch:.4f}, Yaw={yaw:.4f}")
            rospy.loginfo(f"   Score: {score:.2f}")
            rospy.loginfo("")
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("BEST ORIENTATION:")
        best = results[0]
        rospy.loginfo(f"  {best[0]}")
        rospy.loginfo(f"  Roll={best[1]:.4f}, Pitch={best[2]:.4f}, Yaw={best[3]:.4f}")
        rospy.loginfo(f"  Score: {best[4]:.2f}")
        rospy.loginfo("=" * 70)
        rospy.loginfo("Check images in /tmp/camera_test_angle_*.jpg")
        rospy.loginfo("Use: eog /tmp/camera_test_angle_*.jpg")
        rospy.loginfo("=" * 70)


if __name__ == '__main__':
    try:
        tester = CameraRotationTest()
        tester.test_orientations()
        rospy.loginfo("Test complete!")
    except rospy.ROSInterruptException:
        pass
