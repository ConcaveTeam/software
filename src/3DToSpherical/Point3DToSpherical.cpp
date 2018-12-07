#include "Point3DToSpherical.h"
#include <cmath>

#define _USE_MATH_DEFINES

Point3DToSpherical::Point3DToSpherical()
  : nh("~")
  , pub(nh.advertise<concaveteam::Spherical>("aim3d", 1))
  , point_sub(nh.subscribe("/target3d", 1, &Point3DToSpherical::pointCallback, this))
{
}

void Point3DToSpherical::pointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  double theta = acos((double) msg->y / sqrt((double) ((pow(msg->x, 2) + pow(msg->y, 2) + pow(msg->z, 2)))));
  double phi = atan((double) msg->x / (double) msg->z);
  int azim = theta * 180 / M_PI;
  int polar = phi * 180 / M_PI;
  aim.azimuth = polar;
  aim.polar = azim;
  pub.publish(aim);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point3d_to_spherical");
  Point3DToSpherical point3d_to_spherical;
  ros::spin();
}
