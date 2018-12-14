#include "Point2DToSpherical.h"
#include <cmath>

#define _USE_MATH_DEFINES

Point2DToSpherical::Point2DToSpherical()
  : nh("~")
  , pub(nh.advertise<concaveteam::Spherical>("aim", 1))
  , camera_info_sub(nh.subscribe("/left/camera_info", 1, &Point2DToSpherical::cameraInfoCallback, this))
  , point_sub(nh.subscribe("/target", 1, &Point2DToSpherical::pointCallback, this))
  , height(0)
  , width(0)
{
  nh.param<double>("focal_length", focal_length, 2);
}

void Point2DToSpherical::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  height = msg->height;
  width = msg->width;
}

void Point2DToSpherical::pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  if (width == 0 || height == 0) return;
  aim.azimuth = atan((width / 2 - msg->point.y) / focal_length) * 180 / M_PI;
  aim.polar = (M_PI / 2 - atan((height / 2 - msg->point.z) / focal_length)) * 180 / M_PI;
  pub.publish(aim);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point2d_to_spherical");
  Point2DToSpherical point2d_to_spherical;
  ros::spin();
}
