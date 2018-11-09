/**
 * This file publishes to a video stream on /left_cam/image_raw and
 * publishes a processed image and a point denoting where the moving
 * object is in image coordinates.
 */
#pragma once

#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>

/**
 * Filter the contours and sort them from best to worst.
 * The motion tracker calculates raw deltas between a pair of
 * consecutive frames and uses contours from these deltas and uses
 * this function to process them.
 * Currently, the "best" contour is the largest one.
 *
 * @param contours a vector of contours to be filtered and sorted.
 * @param min_area the minimum area a contour can have to be
 * considered.
 * @return a vector of contours filtered and sorted from best to worst.
 */
std::vector<std::vector<cv::Point>> filter_contours(std::vector<std::vector<cv::Point>> contours, double min_area);

/**
 * A class to keep track of subscribing and publishing data, as well
 * as data such as the previous frame.
 */
class TrackMono
{
public:
  TrackMono();

private:
  void callback(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  cv_bridge::CvImageConstPtr cv_ptr;

  image_transport::Subscriber sub;
  image_transport::Publisher pub_img;
  ros::Publisher pub;

  // The previous image
  cv::Mat img0;

  // The location of the best moving object in image coordinates.
  geometry_msgs::PointStamped pub_msg;
};
