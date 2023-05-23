#pragma once

#include <thread>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <nmea_msgs/msg/gpgga.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <llh_converter/height_converter.hpp>
#include <llh_converter/llh_converter.hpp>

class GnssCompass : public rclcpp::Node
{
public:
  GnssCompass();
  ~GnssCompass();

private:
  struct xyzts
  {
    double x;
    double y;
    double z;
    double time;
    bool status = false;
  };

  void callbackMainGga(const nmea_msgs::msg::Gpgga::ConstSharedPtr  msg_ptr);
  void callbackSubGga(const nmea_msgs::msg::Gpgga::ConstSharedPtr  msg_ptr);
  void callbackMainFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr  msg_ptr);
  void callbackSubFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr  msg_ptr);

  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr maingga_sub_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr subgga_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr mainfix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subfix_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr illigal_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;


  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;

  xyzts main_pos_, previous_main_pos_;

  std_msgs::msg::Header main_antenna_header_;

  nav_msgs::msg::Odometry odom_msg_;

  llh_converter::LLHConverter llh_converter_;
  llh_converter::LLHParam lc_param_;

  std::thread diagnostic_thread_;

  std::string map_frame_, base_frame_;
  bool use_change_of_sensor_frame_;
  std::string sensor_frame_;
  std::map<std::string, std::string> key_value_stdmap_;

  int skipping_publish_num_;

  void publishTF(const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose_msg);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr);

  void ggall2fixll(const nmea_msgs::msg::Gpgga::ConstSharedPtr & gga_msg_ptr, double & lat, double & lon);

  void timerDiagnostic();

  double toSec(const std_msgs::msg::Header &msg);

  double calcYaw(const xyzts & main_pos, const xyzts & previous_main_pos, const xyzts & sub_pos, double & baseline_length);

  void processGnss(const xyzts & main_pos, const xyzts & previous_main_pos, const xyzts & sub_pos,
    std_msgs::msg::Header main_antenna_header);

  // param
  double gnss_frequency_;
  int min_gnss_status_;
  int max_gnss_status_;
  double fix_covariance_thershold_;
  double time_thresshold_;
  double yaw_bias_;
  bool use_simple_roswarn_;
  bool use_beseline_outlier_detection_;
  double beseline_length_;
  double allowable_beseline_length_error_;
  int max_skipping_publish_num_;
};
