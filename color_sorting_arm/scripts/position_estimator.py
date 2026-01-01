#!/usr/bin/env python3
"""
Position Estimator Node
Converts 2D pixel coordinates to 3D world coordinates using TF and camera info
"""

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import CameraInfo
from color_sorting_arm.msg import DetectedObject, DetectedObjectArray, Object3D, Object3DArray


class PositionEstimator:
    def __init__(self):
        rospy.init_node('position_estimator', anonymous=True)
        
        self.tf_listener = tf.TransformListener()
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Assumed height of table surface 
        self.table_height = 0.8  # Table top is at z=0.8
        
        # Robot base is at z=0.8 (on table)
        self.robot_base_height = 0.8
        
        # Publishers
        self.position_pub = rospy.Publisher('/object_positions', Object3DArray, queue_size=10)
        
        # Subscribers
        self.detection_sub = rospy.Subscriber('/detected_objects', DetectedObjectArray, 
                                              self.detection_callback)
        self.camera_info_sub = rospy.Subscriber('/arm_camera/camera_info', CameraInfo, 
                                                self.camera_info_callback)
        
        rospy.loginfo("Position Estimator Node initialized")
    
    def camera_info_callback(self, msg):
        """Store camera calibration info"""
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera info received")
    
    def pixel_to_3d(self, px, py, depth=1.0):
        """Convert pixel coordinates to 3D point in camera frame"""
        if self.camera_matrix is None:
            return None
        
        # Intrinsic parameters
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Convert to normalized image coordinates
        x = (px - cx) / fx
        y = (py - cy) / fy
        
        # 3D point in camera frame (using estimated depth)
        point_camera = np.array([x * depth, y * depth, depth])
        
        return point_camera
    
    def estimate_depth(self):
        """
        Estimate depth to table surface using TF
        """
        try:
            # Get transform from camera to base_link
            (trans, rot) = self.tf_listener.lookupTransform('base_link', 
                                                            'camera_optical_frame', 
                                                            rospy.Time(0))
            
            # Camera position relative to base
            camera_z = trans[2]
            camera_x = trans[0]
            
            # Cubes are on table at z=0.83 (world frame)
            # Robot base is at z=0.8 (world frame)
            # So cubes are at z=0.03 relative to robot base
            cube_height_rel = 0.03  # Cube top relative to robot base
            
            # Depth estimation based on camera angle
            # Camera is tilted down, so depth = distance along camera's z-axis
            depth = np.sqrt(camera_x**2 + (camera_z - cube_height_rel)**2)
            
            rospy.loginfo_throttle(5, f"Estimated depth: {depth:.3f}m")
            return max(depth, 0.2)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"TF Error: {e}")
            return 0.4  # Default depth
    
    def transform_to_base_link(self, point_camera):
        """Transform point from camera frame to base_link frame"""
        try:
            # Create PointStamped in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = "camera_optical_frame"
            point_stamped.header.stamp = rospy.Time(0)
            point_stamped.point.x = point_camera[0]
            point_stamped.point.y = point_camera[1]
            point_stamped.point.z = point_camera[2]
            
            # Transform to base_link
            point_base = self.tf_listener.transformPoint('base_link', point_stamped)
            
            return point_base.point
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF Transform Error: {e}")
            return None
    
    def detection_callback(self, msg):
        """Process detected objects and estimate 3D positions"""
        if self.camera_matrix is None:
            rospy.logwarn("Camera info not received yet")
            return
        
        # Wait for TF to be ready
        try:
            self.tf_listener.waitForTransform('base_link', 'camera_optical_frame', 
                                             rospy.Time(0), rospy.Duration(1.0))
        except tf.Exception as e:
            rospy.logwarn(f"TF not ready: {e}")
            return
        
        # Estimate depth to objects
        depth = self.estimate_depth()
        
        object_array = Object3DArray()
        object_array.header = msg.header
        
        for obj in msg.objects:
            # Convert pixel to 3D in camera frame
            point_camera = self.pixel_to_3d(obj.pixel_x, obj.pixel_y, depth)
            
            if point_camera is not None:
                # Transform to base_link
                point_base = self.transform_to_base_link(point_camera)
                
                if point_base is not None:
                    # Create 3D object message
                    obj_3d = Object3D()
                    obj_3d.color = obj.color
                    obj_3d.position = point_base
                    obj_3d.area = obj.area
                    
                    object_array.objects.append(obj_3d)
                    
                    rospy.loginfo(f"Detected {obj.color} cube at ({point_base.x:.2f}, "
                                f"{point_base.y:.2f}, {point_base.z:.2f})")
        
        # Publish 3D positions
        if len(object_array.objects) > 0:
            self.position_pub.publish(object_array)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()


if __name__ == '__main__':
    try:
        estimator = PositionEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass
