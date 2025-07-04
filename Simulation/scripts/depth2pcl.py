#!/usr/bin/env python3
'''
@author: [Debrup, Aman]
'''

'''
This script converts depth info into pointcloud and publishes it onto '/camera/depth/lane_only_pcl'
'''

import rospy
import cv2
import std_msgs.msg
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from utils import *
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh

class Depth2Pcl:
    def __init__(self):
        rospy.init_node('depth_2_pcl')

        # depth frame
        self.depth_frame=None

        # camera params
        self.camera_factor=0
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0

        # ros msgs
        self.bridge=CvBridge()
        self.pcl_msg=PointCloud2()
        self.pcl_o3d=o3d.geometry.PointCloud()
        
        #setup ros message
        self.h=std_msgs.msg.Header()
        self.h.stamp=rospy.Time.now()
        self.h.frame_id="camera_depth_frame"
        self.pcl_msg.header=self.h
        self.field=[
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)]

        self.rate=rospy.Rate(10)
        
        # subscribers
        self.depth_sub=rospy.Subscriber('/camera/depth/lane_only_depth', Image, self.depth_callback)
        self.info_sub=rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.info_callback)

        # publisher
        self.pub=rospy.Publisher('/camera/depth/lane_only_pcl', PointCloud2, queue_size=10)

        while not rospy.is_shutdown():
            if self.depth_frame is not None:
                h, w = self.depth_frame.shape

                ii = np.repeat(range(h), w)
                jj = np.tile(range(w), h)

                x = (jj - self.cx) / self.fx
                y = (ii - self.cy) / self.fy
                length=h*w

                z=self.depth_frame.reshape(length)

                pcd = np.dstack((x * z, y * z, z)).reshape((length, 3))
                self.pcl_o3d.points=o3d.utility.Vector3dVector(pcd)

                self.pcl_msg=orh.o3dpc_to_rospc(self.pcl_o3d)
                self.pcl_msg.header=self.h

                self.pub.publish(self.pcl_msg)
                # self.rate.sleep()
                rospy.loginfo('Publishing point cloud')

            if cv2.waitKey(27) & 0xFF == ord('q'):
                rospy.signal_shutdown('user command')

    def depth_callback(self, data):
        self.depth_frame=self.bridge.imgmsg_to_cv2(data, "32FC1")

    def info_callback(self, camera_info):    
        self.camera_factor=1
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]

        
if __name__=='__main__':
    dp=Depth2Pcl()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
