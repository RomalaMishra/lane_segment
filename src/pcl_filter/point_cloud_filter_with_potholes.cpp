#include <iostream>
#include <iomanip>
#include <chrono>
#include <random>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#define RATE 15

ros::Publisher pub;
ros::Rate *loop_rate;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_potholeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void lanepointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {\

  // std::cout << "pointCloudCallback called\n";
  auto start = std::chrono::high_resolution_clock::now();

  // Convert PointCloud2 message to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZRGB> lane_cloud;
  sensor_msgs::PointCloud2 output;
  pcl::fromROSMsg(*msg, lane_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_laneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  for (size_t i = 0; i < lane_cloud.size(); ++i)
  {
    pcl::PointXYZRGB point_lane = lane_cloud[i];
    if (point_lane.r != 0 || point_lane.g != 0 || point_lane.b != 0)
    {
      filtered_laneCloud->push_back(point_lane);
      indices->indices.push_back(i);
    }
  }

  // Save pcl::PointCloud to PCD file
  // pcl::io::savePCDFileASCII("cuboid.pcd", *cuboidCloud);
  // ROS_INFO("PCD file saved.");

  pcl::toROSMsg(*filtered_laneCloud + *filtered_potholeCloud, output); // Code for concatenation of two point clouds if needed
  // pcl::toROSMsg(*filtered_laneCloud, output);
  // pcl::toROSMsg(*filtered_potholeCloud, output);
  output.header.frame_id = "camera_depth_optical_frame"; //Change frame_id 
  pub.publish(output);
  filtered_potholeCloud->clear();

  //Fps calculation code

  auto stop = std::chrono::high_resolution_clock::now();
  std::cout << std::setprecision(3)
            << "Running at "
            << 1e9 / std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count()
            << "fps\n";
            
  loop_rate->sleep();
}

void potholepointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
  filtered_potholeCloud->clear();
  pcl::PointCloud<pcl::PointXYZRGB> pothole_cloud;
  pcl::fromROSMsg(*msg, pothole_cloud);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_potholeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  for (size_t i = 0; i < pothole_cloud.size(); ++i)
  {
    pcl::PointXYZRGB point_pothole = pothole_cloud[i];
    if (point_pothole.r != 0 || point_pothole.g != 0 || point_pothole.b != 0)
    {
      filtered_potholeCloud->push_back(point_pothole);
      indices->indices.push_back(i);
    }
  }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_filterer"); //Node initialization
  ros::NodeHandle nh;
  
  loop_rate = new ros::Rate(RATE); 

  pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points_marked_potholes", 1, pointcloudCallback);
  ros::Subscriber sub_lane = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points_marked", 1, lanepointcloudCallback);
  ros::Subscriber sub_pothole = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points_marked_potholes", 1, potholepointcloudCallback);

  ros::spin();

  return 0;
}





