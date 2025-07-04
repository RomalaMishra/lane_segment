#!/bin/bash

roslaunch astra_camera astra_pro.launch
rosrun igvc_perception lane_predict.py & 
roslaunch igvc_perception node_exe_unet.launch &
rosrun igvc_perception point_cloud_filter &
rosrun rviz rviz -d /home/romala/catkin_ws/src/igvc_ros/igvc_perception/rviz/igvc_perception.rviz &
