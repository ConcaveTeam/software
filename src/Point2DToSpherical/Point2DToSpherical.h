#pragma once

#include <concaveteam/Spherical.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

class Point2DToSpherical
{
public:
  Point2DToSpherical();

private:
  void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber camera_info_sub;
  ros::Subscriber point_sub;

  double focal_length;
  concaveteam::Spherical aim;
  unsigned int height;
  unsigned int width;
};
