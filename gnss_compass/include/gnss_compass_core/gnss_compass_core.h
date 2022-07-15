#pragma once

#include <ros/ros.h>

#include <nmea_msgs/Gpgga.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "llh_converter/height_converter.hpp"
#include "llh_converter/llh_converter.hpp"

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
  ros::Publisher illigal_odom_pub_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  nmea_msgs::Gpgga::ConstPtr maingga_msg_ptr_, subgga_msg_ptr_;

  nav_msgs::Odometry odom_msg_;

  llh_converter::LLHConverter llh_converter_;
  llh_converter::LLHParam lc_param_;

  void publishTF(const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::PoseStamped & pose_msg);

  void ggall2fixll(const nmea_msgs::Gpgga::ConstPtr & gga_msg_ptr, double & lat, double & lon);

  struct xyz
  {
    double x;
    double y;
    double z;
  };

  // param
  int min_gnss_status_;
  int max_gnss_status_;
  double time_thresshold_;
  double yaw_bias_;
};
