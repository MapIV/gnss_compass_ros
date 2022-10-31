#pragma once

#include <thread>
#include <map>

#include <ros/ros.h>

#include <nmea_msgs/Gpgga.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/DiagnosticArray.h>

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
  ros::Publisher pose_with_covariance_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher illigal_odom_pub_;
  ros::Publisher diagnostics_pub_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  nmea_msgs::Gpgga::ConstPtr maingga_msg_ptr_, previous_maingga_msg_ptr_;

  nav_msgs::Odometry odom_msg_;

  llh_converter::LLHConverter llh_converter_;
  llh_converter::LLHParam lc_param_;

  std::thread diagnostic_thread_;

  std::string map_frame_, base_frame_;
  bool use_mgrs_, use_change_of_sensor_frame_;
  int plane_num_;
  std::string sensor_frame_;
  std::map<std::string, std::string> key_value_stdmap_;

  int skipping_publish_num_;

  void publishTF(const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::PoseStamped & pose_msg);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);

  void ggall2fixll(const nmea_msgs::Gpgga::ConstPtr & gga_msg_ptr, double & lat, double & lon);

  void timerDiagnostic();

  struct xyz
  {
    double x;
    double y;
    double z;
  };

  // param
  double gnss_frequency_;
  int min_gnss_status_;
  int max_gnss_status_;
  double time_thresshold_;
  double yaw_bias_;
  bool use_simple_roswarn_;
  double beseline_length_;
  double allowable_beseline_length_error_;
  int max_skipping_publish_num_;
};
