#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image as ros_img
from cv_bridge import CvBridge
import numpy as np

class PotholeDetector:

    def __init__(self):

        rospy.init_node('pothole_detector', anonymous=True)

        # Subscribe to image topics
        rospy.Subscriber("/camera/color/image_raw", ros_img, self.img_updater)

        # Create publishers for lane markers
        self.pub = rospy.Publisher('/camera/color/potholes_detected', ros_img, queue_size=1)

        self.bridge = CvBridge()

        self.rgb_image = None
        self.time_stamp = None
        self.img_frame_id = "camera_depth_optical_frame"

        rate = rospy.Rate(10)
        # x=1

        while not rospy.is_shutdown():
            if self.rgb_image is None:
                # print(f"passing {x}")
                # x+=1
                # return
                continue
            # Process the image and publish lane markers
            self.process_image(self.rgb_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()

    def img_updater(self, data):

    # Callback function for updating the RGB image
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.time_stamp = data.header.stamp
        self.rgb_image = img

    def process_image(self, imgs):

        ret,img_thresh = cv2.threshold(imgs,230,255,cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8) 

        img_erosion = cv2.erode(img_thresh, kernel, iterations=1) 
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=1) 
        cv2.imshow("frame_processed" , img_dilation)
        cv2.imshow("frame" , imgs)

        imgs_ros = self.bridge.cv2_to_imgmsg(img_dilation, encoding="bgr8")

        imgs_ros.header.stamp = self.time_stamp

        imgs_ros.header.frame_id  = self.img_frame_id

        self.pub.publish(imgs_ros)


if __name__ == '__main__':
    try:
        PotholeDetector()
    except rospy.ROSInterruptException:
        pass