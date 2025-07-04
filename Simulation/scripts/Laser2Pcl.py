#!/usr/bin/env python3
'''
@author: Debrup
'''

'''
This script converts laserscan info into pointcloud
'''

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        rospy.init_node("Laser_to_pcl")
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher('/camera/depth/obs_only_pcl', pc2, queue_size=10)
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def laser_callback(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        self.pcPub.publish(cloud_out)

if __name__ == '__main__':
    rospy.loginfo("PointCloud generated")
    l2pc = Laser2PC()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
