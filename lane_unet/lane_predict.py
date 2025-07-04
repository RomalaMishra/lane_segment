#!/usr/bin/env python3

from __future__ import print_function

# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from .utils.model import unet  

# LaneDetector class definition
class LaneDetector:

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('lane_detector', anonymous=True)

        # Create a CvBridge instance to convert between OpenCV and ROS image formats
        self.bridge = CvBridge()

        # Subscribe to RGB image and depth image topics
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Publish processed image to a new topic
        self.image_pub = rospy.Publisher("/camera/color/lane_markers_unet", Image, queue_size=10)
        
        # Load the UNet model for lane detection
        self.model = unet('/home/romala/catkin_ws/src/igvc_ros/igvc_perception/lane_unet/lane_model.h5')
        self.model.make_predict_function()

        # Initialize variables
        self.rgb_image = None
        self.time_stamp = None
        self.rgb_image_frame_id = "camera_depth_optical_frame"

        # Set the rate for the ROS node loop
        rate = rospy.Rate(100)

        # Main ROS node loop
        while not rospy.is_shutdown():
            
            # Continue looping until RGB image is received
            if self.rgb_image is None:
                continue

            # Process the RGB image for lane detection
            self.predict_lane(self.rgb_image)

            # Sleep according to the specified rate
            rate.sleep()

    def callback(self, data):
        # Callback function for RGB image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.time_stamp = data.header.stamp
        self.rgb_image = cv_image
        
    def depth_callback(self, data):
        # Callback function for depth image
        depth_image = self.bridge.imgmsg_to_cv2(data, "8UC1")
        # Additional processing for depth image can be added here
        
    def predict_lane(self, img):
        # Function to predict lanes in the given RGB image
        
        # Resize and reshape the image for the model input
        cv_image = cv2.resize(img, (128, 128))
        transformed_cv_image = cv_image.reshape(1, 128, 128, 3)

        # Use the UNet model for lane prediction
        lane_cv_image = self.model.predict(transformed_cv_image)

        # Post-process the predicted lane image
        lane_cv_image = lane_cv_image.reshape(128, 128)
        lane_cv_image = 255 * (lane_cv_image * 2555555 > 100)
        lane_cv_image = lane_cv_image.astype(np.uint8)
        kernel = np.ones((2, 2), np.uint8)
        lane_cv_image = cv2.dilate(lane_cv_image, kernel, iterations=1)
        lane_cv_image = cv2.resize(lane_cv_image, (640, 480))
        lane_cv_image = 255 * (lane_cv_image > 100)
        lane_cv_image = lane_cv_image.astype(np.uint8)
        
        # Convert lane image to grayscale for display
        gray_image = cv2.cvtColor(lane_cv_image, cv2.COLOR_GRAY2BGR)

        # Convert the lane image to ROS image format and publish
        imgs_ros_rgb = self.bridge.cv2_to_imgmsg(gray_image, 'bgr8')
        imgs_ros_rgb.header.stamp = self.time_stamp
        imgs_ros_rgb.header.frame_id = self.rgb_image_frame_id
        self.image_pub.publish(imgs_ros_rgb)

# Main execution block
if __name__ == '__main__':
    try:
        # Create an instance of LaneDetector
        LaneDetector()

    except rospy.ROSInterruptException:
        pass
