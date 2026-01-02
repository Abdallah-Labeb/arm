#!/usr/bin/env python3
"""
Simple Raw Image Saver - saves exactly what comes from camera
"""

import rospy
from sensor_msgs.msg import Image
import numpy as np


def save_raw_image(msg):
    """Save raw image data directly to file"""
    print(f"\nReceived image:")
    print(f"  Encoding: {msg.encoding}")
    print(f"  Size: {msg.width}x{msg.height}")
    print(f"  Step: {msg.step}")
    print(f"  Data length: {len(msg.data)}")
    
    # Convert data to numpy array
    if msg.encoding == "rgb8":
        # RGB image
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        img_data = img_data.reshape((msg.height, msg.width, 3))
        
        print(f"  Array shape: {img_data.shape}")
        print(f"  Min: {img_data.min()}, Max: {img_data.max()}, Mean: {img_data.mean():.2f}")
        print(f"  Std: {img_data.std():.2f}")
        
        # Save as raw numpy
        filename = "/tmp/raw_camera_image.npy"
        np.save(filename, img_data)
        print(f"  Saved numpy: {filename}")
        
        # Also save as PPM (simple RGB format)
        ppm_file = "/tmp/raw_camera_image.ppm"
        with open(ppm_file, 'wb') as f:
            f.write(f"P6\n{msg.width} {msg.height}\n255\n".encode())
            f.write(img_data.tobytes())
        print(f"  Saved PPM: {ppm_file}")
        print(f"  View with: eog {ppm_file}")
        
    rospy.signal_shutdown("Image saved")


if __name__ == '__main__':
    rospy.init_node('raw_image_saver')
    print("Waiting for ONE image from /camera/image_raw...")
    rospy.Subscriber('/camera/image_raw', Image, save_raw_image)
    rospy.spin()
