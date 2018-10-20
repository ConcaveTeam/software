#include "TrackMono.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

TrackMono::TrackMono()
  : it(nh)
  , sub(it.subscribe("/left/image_raw", 1, &TrackMono::callback, this))
  , pub_img(it.advertise("/monotrack", 1))
  , pub(nh.advertise<geometry_msgs::PointStamped>("target", 1))
{
  pub_msg.header.frame_id = "left_cam";
}

void TrackMono::callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::Mat color = cv_ptr->image;
  cv::Mat img;
  cv::cvtColor(color, img, CV_BGR2GRAY);

  if (img0.empty())
    {
      img0 = img;
      return;
    }

  cv::Mat diff;
  cv::absdiff(img0, img, diff);

  cv::Mat thresh;
  cv::threshold(diff, thresh, 25, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  cv::Mat contourimg;
  color.copyTo(contourimg);
  cv::drawContours(contourimg, contours, 0, cv::Scalar(0, 0, 255));

  std::sort (contours.begin(), contours.end(),
     [](auto a, auto b) {return cv::contourArea(cv::Mat(a)) > cv::contourArea(cv::Mat(b));});

  for (auto i = 0; i < contours.size(); i++)
    {
      cv::drawContours(contourimg, contours, i, cv::Scalar(0, 0, 255));
    }

  img0 = img;
  sensor_msgs::ImagePtr published_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", contourimg).toImageMsg();
  published_image->header.frame_id="left_cam";
  pub_img.publish(published_image);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_mono");
  TrackMono track_mono;
  ros::spin();
}
