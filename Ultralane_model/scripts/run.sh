#!/bin/bash

roslaunch astra_camera astra_pro.launch &
rosrun igvc_perception lane_detect.py ../dl_model/configs/tusimple.py & 
roslaunch depth_image_proc node_exe_igv.launch &
rosrun igvc_perception point_cloud_filter &
rosrun rviz rviz -d /media/warlord/workspace/ros_ws/src/igvc_ros/igvc_perception/rviz/igvc_perception.rviz &
