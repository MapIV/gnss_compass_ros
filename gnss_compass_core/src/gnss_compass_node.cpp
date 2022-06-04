#include <ros/ros.h>

#include "gnss_compass_core/gnss_compass_core.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gnss_compass");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  GnssCompass node(nh, private_nh);

  ros::spin();
  return 0;
}
