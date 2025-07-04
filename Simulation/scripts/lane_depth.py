#!/usr/bin/env python3
'''
@author: Debrup
'''

'''
This script extracts lane info and publishes a lane only depth image
'''

import cv2
from igvc_perception.scripts.utils import andImg, processImage
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import message_filters
from utils import *

class LaneExtract:
    def __init__(self):
        # initialise node
        rospy.init_node('lane_extract')

        # subscriber
        self.rgb_sub=message_filters.Subscriber('camera/color/image_raw', Image)
        self.depth_sub=message_filters.Subscriber('camera/depth/depth_raw', Image)
        self.lane_pub=rospy.Publisher('camera/depth/lane_only_depth', Image, queue_size=10)

        # param init
        self.rgb_frame=None
        self.depth_frame=None

        # mask upper and lower ranges for white
        self.lower_white = np.array([156, 0, 0])
        self.upper_white = np.array([255, 255, 255])

        self.rate=rospy.Rate(10)

        self.bridge=CvBridge()

        #Time synchronizer to work with multiple camera topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.5)
        self.ts.registerCallback(self.image_callback)

        while not rospy.is_shutdown():

            if self.rgb_frame is not None and self.depth_frame is not None: 
                # image processing logic
                
                # thresh = processImage(self.rgb_frame, np.array([h_l, s_l, v_l]), np.array([h_h, s_h, v_h]))
                thresh = processImage(self.rgb_frame, self.lower_white, self.upper_white)
                # cv2.imshow("rgb", self.rgb_frame)
                # cv2.imshow("thresh", thresh)

                #lane only depth image generation
                lane_only_depth_img = andImg(self.depth_frame, thresh)

                self.lane_pub.publish(self.bridge.cv2_to_imgmsg(lane_only_depth_img, '32FC1'))
                rospy.loginfo('Publishing lane only depth')
                # self.rate.sleep()

            if cv2.waitKey(27) & 0xFF == ord('q'):
                rospy.signal_shutdown('user command')

    def image_callback(self, rgb_data, depth_data):
        self.rgb_frame = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")

        self.h, self.w, _ = self.rgb_frame.shape

if __name__ == '__main__':
    le=LaneExtract()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)