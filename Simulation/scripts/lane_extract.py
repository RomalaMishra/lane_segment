#!/usr/bin/env python3

import cv2
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import message_filters
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh

class LaneExtract:
    
    def __init__(self):
        
        # initialise node
        rospy.init_node('lane_extract')

        # subscriber
        self.rgb_sub=message_filters.Subscriber('/camera/color/lane_markers', Image)
        self.depth_sub=message_filters.Subscriber('camera/depth/image_raw', Image)
        self.info_sub=rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.info_callback)

        # publisher
        self.lane_pub=rospy.Publisher('/camera/depth/lane_pcl', PointCloud2, queue_size=10)

        # param init
        self.rgb_frame=None
        self.depth_frame=None
        
        # camera params
        self.camera_factor=0
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0

        self.rate=rospy.Rate(10)

        # ros msgs
        self.bridge=CvBridge()
        self.pcl_msg=PointCloud2()
        self.pcl_o3d=o3d.geometry.PointCloud()
        
        #setup ros message
        self.lane_pcl=std_msgs.msg.Header()
        self.lane_pcl.stamp=rospy.Time.now()
        self.lane_pcl.frame_id="camera_depth_frame"
        self.pcl_msg.header=self.lane_pcl
        self.field=[
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)]


        #Time synchronizer to work with multiple camera topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.5)
        self.ts.registerCallback(self.image_callback)

        while not rospy.is_shutdown():

            if self.rgb_frame is not None and self.depth_frame is not None:     
                
                #lane only depth image generation
                lane_only_depth_img = self.andImg(self.depth_frame, self.rgb_frame)
                
                ii = np.repeat(range(self.h), self.w)
                jj = np.tile(range(self.w), self.h)

                x = (jj - self.cx) / self.fx
                y = (ii - self.cy) / self.fy
                length=self.h*self.w

                z=lane_only_depth_img.reshape(length)

                pcd = np.dstack((x * z, y * z, z)).reshape((length, 3))
                self.pcl_o3d.points=o3d.utility.Vector3dVector(pcd)

                self.pcl_msg=orh.o3dpc_to_rospc(self.pcl_o3d)
                self.pcl_msg.header=self.lane_pcl
                
                self.lane_pub.publish(self.pcl_msg)
                rospy.loginfo('Publishing point cloud')                

            if cv2.waitKey(27) & 0xFF == ord('q'):
                rospy.signal_shutdown('user command')

    def image_callback(self, rgb_data, depth_data):
        
        self.rgb_frame = self.bridge.imgmsg_to_cv2(rgb_data, '8UC1')
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")

        self.h, self.w = self.depth_frame.shape
        
    def info_callback(self, camera_info):    
        
        self.camera_factor=1
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]
    
    def andImg(self, image, mask):
        
        result = cv2.bitwise_and(image, image, mask = mask)
        return result

if __name__ == '__main__':
    
    le=LaneExtract()
    
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)