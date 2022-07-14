#include "gnss_compass_core/gnss_compass_core.h"

GnssCompass::GnssCompass(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{

  private_nh_.getParam("min_gnss_status", min_gnss_status_);
  private_nh_.getParam("max_gnss_status", max_gnss_status_);
  private_nh_.getParam("time_thresshold", time_thresshold_);
  private_nh_.getParam("yaw_bias", yaw_bias_);

  ROS_INFO(
    "min_gnss_status: %d, max_gnss_status: %d, time_thresshold: %lf, yaw_bias: %lf", min_gnss_status_,
    max_gnss_status_, time_thresshold_, yaw_bias_);

  maingga_sub_ = nh_.subscribe("/main/mosaic/gga", 100, &GnssCompass::callbackMainGga, this);
  subgga_sub_ = nh_.subscribe("/sub/mosaic/gga", 100, &GnssCompass::callbackSubGga, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_compass_pose", 10);
  odom_pub_ =
    nh_.advertise<nav_msgs::Odometry>("gnss_compass_odom", 10);

  // LLHConverter setting
  lc_param_.use_mgrs = true;
  lc_param_.height_convert_type = llh_converter::ConvertType::ORTHO2ELLIPS;
  lc_param_.geoid_type = llh_converter::GeoidType::EGM2008;

}

GnssCompass::~GnssCompass() {}

void GnssCompass::callbackMainGga(const nmea_msgs::Gpgga::ConstPtr & maingga_msg_ptr)
{
  int gps_qual = maingga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  if(!is_gnss_status_ok)
  {
    // ROS_WARN("Main is not fixed. status is %d", maingga_msg_ptr->gps_qual);
    return;
  }

  maingga_msg_ptr_ = maingga_msg_ptr;
}

void GnssCompass::callbackSubGga(const nmea_msgs::Gpgga::ConstPtr & subgga_msg_ptr)
{
  if (maingga_msg_ptr_ == nullptr) {
    ROS_WARN("Main is not subscrubbed.");
    return;
  }
  double dt = maingga_msg_ptr_->header.stamp.toSec() - subgga_msg_ptr->header.stamp.toSec();
  if(std::abs(dt) > time_thresshold_) {
    ROS_WARN("The difference between the main and sub timestamps is too large.");
    return;
  }

  int gps_qual = subgga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  if(!is_gnss_status_ok)
  {
    // ROS_WARN("Sub is not fixed %d", subgga_msg_ptr->gps_qual);
    return;
  }

  xyz main_pos, sub_pos;
  double navsat_lat, navsat_lon;

  ggall2fixll(maingga_msg_ptr_, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, maingga_msg_ptr_->alt,
    main_pos.x, main_pos.y, main_pos.z, lc_param_);

  ggall2fixll(subgga_msg_ptr, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, maingga_msg_ptr_->alt,
    sub_pos.x, sub_pos.y, sub_pos.z, lc_param_);

  double diff_x = sub_pos.x - main_pos.x;
  double diff_y = sub_pos.y - main_pos.y;
  double diff_z = sub_pos.z - main_pos.z; // TODO: length check
  double theta = - std::atan2(diff_x, diff_y) + yaw_bias_;

  tf2::Quaternion localization_quat;
  localization_quat.setRPY(0, 0, theta);
  geometry_msgs::Quaternion quat = tf2::toMsg(localization_quat);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = maingga_msg_ptr_->header;
  pose_msg.header.frame_id = "map";
  pose_msg.pose.position.x = main_pos.x;
  pose_msg.pose.position.y = main_pos.y;
  pose_msg.pose.position.z = main_pos.z;
  pose_msg.pose.orientation = quat;
  pose_pub_.publish(pose_msg);

  odom_msg_.header = pose_msg.header;
  odom_msg_.child_frame_id = "main_gnss";
  odom_msg_.pose.pose = pose_msg.pose;
  odom_pub_.publish(odom_msg_);

}

void GnssCompass::ggall2fixll(const nmea_msgs::Gpgga::ConstPtr & gga_msg_ptr, double & navsat_lat, double & navsat_lon)
{
  if (gga_msg_ptr->lat_dir == "N") {
    navsat_lat = gga_msg_ptr->lat;
  } else if (gga_msg_ptr->lat_dir == "S") {
    navsat_lat = -gga_msg_ptr->lat;
  }
  if (gga_msg_ptr->lon_dir == "E") {
    navsat_lon = gga_msg_ptr->lon;
  } else if (gga_msg_ptr->lon_dir == "W") {
    navsat_lon = -gga_msg_ptr->lon;
  }
}
