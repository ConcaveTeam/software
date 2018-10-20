#include "geometry_msgs/PointStamped.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class TrackMono
{
public:
  TrackMono();

private:
  void callback(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  image_transport::Publisher pub_img;
  ros::Publisher pub;
  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat img0;
  geometry_msgs::PointStamped pub_msg;
};
