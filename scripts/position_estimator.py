#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo
from color_sorting_arm.msg import DetectedObject, DetectedObjectArray, Object3D, Object3DArray
import tf2_geometry_msgs


class PositionEstimator:
    
    TABLE_HEIGHT = 0.725
    
    def __init__(self):
        rospy.init_node('position_estimator')
        rospy.loginfo("Starting Position Estimator...")
        
        self.camera_matrix = None
        self.camera_frame = 'camera_optical_frame'
        self.base_frame = 'base_link'
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.position_pub = rospy.Publisher('/object_positions', Object3DArray, queue_size=10)
        
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/detected_objects', DetectedObjectArray, self.detection_callback)
        
        rospy.loginfo("Position Estimator initialized")
    
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.image_width = msg.width
            self.image_height = msg.height
            rospy.loginfo(f"Camera info received: {msg.width}x{msg.height}")
            rospy.loginfo(f"Camera matrix:\n{self.camera_matrix}")
    
    def pixel_to_3d_simple(self, pixel_x, pixel_y, depth=0.5):
        if self.camera_matrix is None:
            rospy.logwarn("Camera matrix not yet available")
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth
        
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        
        return point
    
    def estimate_depth_from_area(self, area):
        reference_area = 2000.0
        reference_depth = 0.5
        
        if area <= 0:
            return reference_depth
        
        depth = reference_depth * np.sqrt(reference_area / area)
        
        depth = np.clip(depth, 0.2, 1.0)
        
        return depth
    
    def transform_point_to_base(self, point, source_frame):
        try:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = rospy.Time(0)
            point_stamped.point = point
            
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            return transformed.point
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return None
    
    def estimate_position_with_known_height(self, pixel_x, pixel_y, target_z):
        if self.camera_matrix is None:
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logwarn(f"Cannot get camera transform: {e}")
            return None
        
        cam_pos = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        ray_cam = np.array([
            (pixel_x - cx) / fx,
            (pixel_y - cy) / fy,
            1.0
        ])
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        
        q = transform.transform.rotation
        R = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        
        ray_base = R @ ray_cam
        
        if abs(ray_base[2]) < 1e-6:
            rospy.logwarn("Ray is parallel to table plane")
            return None
        
        t = (target_z - cam_pos[2]) / ray_base[2]
        
        if t < 0:
            rospy.logwarn("Target is behind camera")
            return None
        
        point = Point()
        point.x = cam_pos[0] + t * ray_base[0]
        point.y = cam_pos[1] + t * ray_base[1]
        point.z = target_z
        
        return point
    
    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        return R
    
    def detection_callback(self, msg):
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5, "Waiting for camera info...")
            return
        
        output = Object3DArray()
        output.header = msg.header
        output.header.frame_id = self.base_frame
        
        for det in msg.objects:
            pos = self.estimate_position_with_known_height(
                det.pixel_x, det.pixel_y, self.TABLE_HEIGHT
            )
            
            if pos is None:
                depth = self.estimate_depth_from_area(det.area)
                camera_point = self.pixel_to_3d_simple(det.pixel_x, det.pixel_y, depth)
                
                if camera_point is not None:
                    pos = self.transform_point_to_base(camera_point, self.camera_frame)
            
            if pos is not None:
                obj_3d = Object3D()
                obj_3d.color = det.color
                obj_3d.position = pos
                obj_3d.confidence = 0.8
                output.objects.append(obj_3d)
                
                rospy.loginfo_throttle(2, 
                    f"Detected {det.color} object at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        if output.objects:
            self.position_pub.publish(output)
    
    def run(self):
        rospy.spin()


def main():
    try:
        estimator = PositionEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
