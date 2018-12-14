#include "Track3d.h"
#include <sensor_msgs/image_encodings.h>

cv::Point nearestNonBlack(cv::Mat img, cv::Point p)
{
  cv::Point pCoord;
  std::vector<cv::Point> points;

  for (auto x = 0; x < img.cols; x++)
    for (auto y = 0; y < img.rows; y++)
      if (img.at<uchar>(y, x) != 0)
        points.push_back({x, y});

  return *std::min_element(points.begin(), points.end(), [](auto a, auto b) {return cv::norm(a) < cv::norm(b);});
}

Track3d::Track3d()
  : nh("~")
  , target3d_pub(nh.advertise<stereo_msgs::DisparityImage>("targetDisp", 1))
  , target_sub(nh.subscribe("/target", 1, &Track3d::targetCb, this))
  , disparity_sub(nh.subscribe("/stereo/disparity", 1, &Track3d::disparityCb, this))
{
}

void Track3d::targetCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  target = *msg;
}

void Track3d::disparityCb(const stereo_msgs::DisparityImage::ConstPtr& msg)
{
  const auto dispImg = msg->image;

  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::MONO8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::Mat disparity = cv_ptr->image;
  cv::Point matchedPix = nearestNonBlack(disparity, {target.point.y, target.point.z});
  cv::Mat filtered(disparity.cols, disparity.rows, CV_8UC1, cv::Scalar(0));
  filtered.at<uchar>(matchedPix) = disparity.at<uchar>(matchedPix);

  sensor_msgs::ImagePtr published_disparity = cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered).toImageMsg();
  auto msgCpy = *msg;
  msgCpy.image = *published_disparity;

  target3d_pub.publish(msgCpy);
}

int main()
{
}
