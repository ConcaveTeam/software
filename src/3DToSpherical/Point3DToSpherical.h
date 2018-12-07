#pragma once

#include <concaveteam/Spherical.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

class Point3DToSpherical
{
 public:
  Point3DToSpherical();

 private:
  void pointCallback(const geometry_msgs::Point::ConstPtr& msg);

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber point_sub;

  concaveteam::Spherical aim;
  unsigned int height;
  unsigned int width;
};
