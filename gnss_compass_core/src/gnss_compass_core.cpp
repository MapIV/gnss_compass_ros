#include "gnss_compass_core/gnss_compass_core.h"

GnssCompass::GnssCompass(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{

  maingga_sub_ = nh_.subscribe("vehicle/twist", 100, &GnssCompass::callbackMainGga, this);
  subgga_sub_ = nh_.subscribe("imu", 100, &GnssCompass::callbackSubGga, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
  odom_pub_ =
    nh_.advertise<nav_msgs::Odometry>("odom", 10);

}

GnssCompass::~GnssCompass() {}

void GnssCompass::callbackMainGga(const nmea_msgs::Gpgga::ConstPtr & maingga_msg_ptr)
{
  maingga_msg_ptr_ = maingga_msg_ptr;
}

void GnssCompass::callbackSubGga(const nmea_msgs::Gpgga::ConstPtr & subgga_msg_ptr)
{
  if (maingga_msg_ptr_ == nullptr) {
    return;
  }
}
