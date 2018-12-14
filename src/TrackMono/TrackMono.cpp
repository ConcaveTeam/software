#include "TrackMono.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

std::vector<std::vector<cv::Point>> filter_contours(std::vector<std::vector<cv::Point>> contours, double min_area)
{
  std::sort(contours.begin(), contours.end(),
            [](auto a, auto b) {return cv::contourArea(cv::Mat(a)) > cv::contourArea(cv::Mat(b));});

  std::vector<std::vector<cv::Point>> filtered;
  copy_if(contours.begin(), contours.end(), back_inserter(filtered),
          [min_area](auto x) {return cv::contourArea(cv::Mat(x)) > min_area;});

  return filtered;
}

void TrackMono::callback_left(const sensor_msgs::ImageConstPtr& msg)
{

  // Convert the image message to a cv::Mat
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

  // Handle the case on initialization when a previous frame has not
  // been created.
  if (img0_left.empty())
    {
      img0_left = img;
      return;
    }

  // Get the delta between the last and current frame.
  cv::Mat delta;
  cv::absdiff(img0_left, img, delta);

  cv::Mat thresh;
  cv::threshold(delta, thresh, 25, 255, cv::THRESH_BINARY);

  // Get the best contours from the delta image.
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  contours = filter_contours(contours, 255);

  // Find the centers of the moving objects
  std::vector<cv::Point> centers;
  std::vector<double> radii;

  cv::Point2f center;
  float radius;

  for (size_t i = 0; i < contours.size(); i++)
    {
      cv::minEnclosingCircle(contours[i], center, radius);
      centers.push_back(center);
      radii.push_back(radius);
    }

  // Draw visual indicators on a new image to be published.
  cv::Mat contourimg;

  color.copyTo(contourimg);

  for (size_t i = 0; i < contours.size(); i++)
    {
      cv::drawContours(contourimg, contours, i, cv::Scalar(255, 0, 0));
      circle(contourimg, centers[i], radii[i], cv::Scalar(0, 0, 255));
    }

  if (!centers.empty())
    {
      left_saved.point.y = centers[0].x;
      left_saved.point.z = centers[0].y;
      left_saved.header.stamp = ros::Time::now();
    }

  // Finish this iteration by updating the previous image and
  // publishing the data.
  img0_left = img;

  sensor_msgs::ImagePtr published_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", contourimg).toImageMsg();
  dist1=0.1;//
  distance between camerasvirtual_depth=831;
  cam_angular_width = 1.0472;


  right_saved.azimuth = atan((width / 2 - right_saved.point.y) / virtual_depth) ;

  right_saved.polar = (M_PI / 2 - atan((height / 2 - right_saved.point.z) / virtual_depth)) * 180 / M_PI;
  left_saved.azimuth = atan((width / 2 - left_saved.point.y) / virtual_depth) ;
  left_saved.polar = (M_PI / 2 - atan((height / 2 - left_saved.point.z) / virtual_depth)) * 180 / M_PI;
  angle3= (M_PI)-(left_saved.azimuth+right_saved.azimuth);
  distance_from_left= (dist1*sin(right_saved.azimuth))/sin(angle3);




}
TrackMono::TrackMono()
        : it(nh)
        , sub_right(it.subscribe("/stereo/right/image_rect", 1, &TrackMono::callback_right, this))
        , sub_left(it.subscribe("/stereo/left/image_rect", 1, &TrackMono::callback_left, this))
        , pub_img(it.advertise("monotrack", 1))
        , pub(nh.advertise<geometry_msgs::PointStamped>("target", 1))
{
    pub_msg.header.frame_id = "right_cam";
}
void TrackMono::callback_right(const sensor_msgs::ImageConstPtr& msg)
{

    // Convert the image message to a cv::Mat
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

    // Handle the case on initialization when a previous frame has not
    // been created.
    if (img0_right.empty())
    {
        img0_right = img;
        return;
    }

    // Get the delta between the last and current frame.
    cv::Mat delta;
    cv::absdiff(img0_right, img, delta);

    cv::Mat thresh;
    cv::threshold(delta, thresh, 25, 255, cv::THRESH_BINARY);

    // Get the best contours from the delta image.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    contours = filter_contours(contours, 255);

    // Find the centers of the moving objects
    std::vector<cv::Point> centers;
    std::vector<double> radii;

    cv::Point2f center;
    float radius;

    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::minEnclosingCircle(contours[i], center, radius);
        centers.push_back(center);
        radii.push_back(radius);
    }

    // Draw visual indicators on a new image to be published.
    cv::Mat contourimg;

    color.copyTo(contourimg);

    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::drawContours(contourimg, contours, i, cv::Scalar(255, 0, 0));
        circle(contourimg, centers[i], radii[i], cv::Scalar(0, 0, 255));
    }

    if (!centers.empty())
    {
        right_saved.point.y = centers[0].x;
        right_saved.point.z = centers[0].y;
        right_saved.header.stamp = ros::Time::now();
    }

    // Finish this iteration by updating the previous image and
    // publishing the data.
    img0_right = img;

    sensor_msgs::ImagePtr published_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", contourimg).toImageMsg();
    published_image->header.frame_id="left_cam";
    pub_img.publish(published_image);

    pub.publish(pub_msg);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "track_mono");
  TrackMono track_mono;
  ros::spin();
  dist1=0.1//distance between cameras
  virtual_depth=831;
  cam_angular_width = 1.0472;


  //right_saved.azimuth = atan((width / 2 - right_saved.point.y) / virtual_depth) ;

  //right_saved.polar = (M_PI / 2 - atan((height / 2 - right_saved.point.z) / virtual_depth)) * 180 / M_PI;
  //left_saved.azimuth = atan((width / 2 - left_saved.point.y) / virtual_depth) ;
  //left_saved.polar = (M_PI / 2 - atan((height / 2 - left_saved.point.z) / virtual_depth)) * 180 / M_PI;
  //angle3= (M_PI)-(left_saved.azimuth+right_saved.azimuth);
  //distance_from_left= (dist1*sin(right_saved.azimuth))/sin(angle3);




}
