#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/core/core.hpp>

cv::Point nearestNonBlack(cv::Mat img, cv::Point p);

class Track3d
{
public:
  Track3d();

private:
  void targetCb(const geometry_msgs::PointStamped::ConstPtr& msg);
  void disparityCb(const stereo_msgs::DisparityImage::ConstPtr& msg);

  cv_bridge::CvImageConstPtr cv_ptr;

  ros::NodeHandle nh;
  ros::Publisher target3d_pub;
  ros::Subscriber target_sub;
  ros::Subscriber disparity_sub;

  geometry_msgs::PointStamped target;
};
