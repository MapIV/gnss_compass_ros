#pragma once

#include <ros/ros.h>

#include <nmea_msgs/Gpgga.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class GnssCompass
{
public:
  GnssCompass(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~GnssCompass();

private:
  void callbackMainGga(const nmea_msgs::Gpgga::ConstPtr & maingga_msg_ptr);
  void callbackSubGga(const nmea_msgs::Gpgga::ConstPtr & subgga_msg_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber maingga_sub_;
  ros::Subscriber subgga_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;

  nmea_msgs::Gpgga::ConstPtr maingga_msg_ptr_, subgga_msg_ptr_;
};
