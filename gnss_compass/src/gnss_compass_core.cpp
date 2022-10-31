#include "gnss_compass_core/gnss_compass_core.h"

GnssCompass::GnssCompass(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh),
  private_nh_(private_nh),
  tf2_listener_(tf2_buffer_)
{
  private_nh_.getParam("map_frame", map_frame_);
  private_nh_.getParam("base_frame", base_frame_);
  private_nh_.getParam("use_mgrs", use_mgrs_);
  private_nh_.getParam("plane_num", plane_num_);
  private_nh_.getParam("use_change_of_sensor_frame", use_change_of_sensor_frame_);
  private_nh_.getParam("sensor_frame", sensor_frame_);
  private_nh_.getParam("gnss_frequency", gnss_frequency_);
  private_nh_.getParam("min_gnss_status", min_gnss_status_);
  private_nh_.getParam("max_gnss_status", max_gnss_status_);
  private_nh_.getParam("time_thresshold", time_thresshold_);
  private_nh_.getParam("yaw_bias", yaw_bias_);
  private_nh_.getParam("use_simple_roswarn", use_simple_roswarn_);
  private_nh_.getParam("beseline_length", beseline_length_);
  private_nh_.getParam("allowable_beseline_length_error", allowable_beseline_length_error_);
  private_nh_.getParam("max_skipping_publish_num", max_skipping_publish_num_);

  ROS_INFO("base_frame_id: %s", base_frame_.c_str());
  if(use_change_of_sensor_frame_)
  {
    ROS_INFO("sensor_frame_id: %s", sensor_frame_.c_str());
  }
  ROS_INFO("min_gnss_status: %d, max_gnss_status: %d, time_thresshold: %lf, yaw_bias: %lf", min_gnss_status_,
    max_gnss_status_, time_thresshold_, yaw_bias_);
  ROS_INFO("use_simple_roswarn: %d, beseline_length: %lf, allowable_beseline_length_error: %lf, max_skipping_publish_num: %d", use_simple_roswarn_,
    beseline_length_, allowable_beseline_length_error_, max_skipping_publish_num_);

  maingga_sub_ = nh_.subscribe("main_gnss_gga", 100, &GnssCompass::callbackMainGga, this);
  subgga_sub_ = nh_.subscribe("sub_gnss_gga", 100, &GnssCompass::callbackSubGga, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_compass_pose", 10);
  pose_with_covariance_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("gnss_compass_pose_with_covariance", 10);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("gnss_compass_odom", 10);
  illigal_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("illigal_gnss_compass_odom", 10);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

  diagnostic_thread_ = std::thread(&GnssCompass::timerDiagnostic, this);
  diagnostic_thread_.detach();

  // LLHConverter setting
  lc_param_.use_mgrs = use_mgrs_;
  lc_param_.plane_num = plane_num_;
  lc_param_.height_convert_type = llh_converter::ConvertType::NONE;
  lc_param_.geoid_type = llh_converter::GeoidType::EGM2008;

  skipping_publish_num_ = 0;
}

GnssCompass::~GnssCompass() {}

void GnssCompass::callbackMainGga(const nmea_msgs::Gpgga::ConstPtr & maingga_msg_ptr)
{
  int gps_qual = maingga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  if(!is_gnss_status_ok)
  {
    if(!use_simple_roswarn_) ROS_WARN("Main is not fixed. status is %d", maingga_msg_ptr->gps_qual);
    return;
  }
  if(maingga_msg_ptr != nullptr)
  {
    previous_maingga_msg_ptr_ = maingga_msg_ptr_;
  }
  maingga_msg_ptr_ = maingga_msg_ptr;

}

void GnssCompass::callbackSubGga(const nmea_msgs::Gpgga::ConstPtr & subgga_msg_ptr)
{
  if (maingga_msg_ptr_ == nullptr || previous_maingga_msg_ptr_ == nullptr) {
    if(!use_simple_roswarn_) ROS_WARN("Main is not subscrubbed.");
    skipping_publish_num_++;
    return;
  }

  double t_pm = previous_maingga_msg_ptr_->header.stamp.toSec();
  double t_m = maingga_msg_ptr_->header.stamp.toSec();
  double dt_mm = t_pm - t_m;
  if(std::abs(dt_mm) > 1.5 * 1.0 / gnss_frequency_) { // Up to 150% allowed
    if(!use_simple_roswarn_) ROS_WARN("The difference between timestamps is large:dt_mm %lf.", dt_mm);
    skipping_publish_num_++;
    return;
  }

  double t_s = subgga_msg_ptr->header.stamp.toSec();
  double dt_ms = t_s - t_m;
  if(std::abs(dt_ms) > time_thresshold_ || dt_ms < 0) {
    if(!use_simple_roswarn_) ROS_WARN("The difference between timestamps is large:dt_ms %lf.", dt_ms);
    skipping_publish_num_++;
    return;
  }

  int gps_qual = subgga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  if(!is_gnss_status_ok)
  {
    if(!use_simple_roswarn_) ROS_WARN("Sub is not fixed %d", subgga_msg_ptr->gps_qual);
    skipping_publish_num_++;
    return;
  }

  skipping_publish_num_ = 0;

  xyz main_pos, previous_main_pos, sub_pos;
  double navsat_lat, navsat_lon, previous_navsat_lat, previous_navsat_lon;

  ggall2fixll(maingga_msg_ptr_, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, maingga_msg_ptr_->alt,
    main_pos.x, main_pos.y, main_pos.z, lc_param_);

  ggall2fixll(previous_maingga_msg_ptr_, previous_navsat_lat, previous_navsat_lon);
  llh_converter_.convertDeg2XYZ(previous_navsat_lat, previous_navsat_lon, previous_maingga_msg_ptr_->alt,
    previous_main_pos.x, previous_main_pos.y, previous_main_pos.z, lc_param_);

  // interpotation: x = a * t + b
  // m:main, pm:previous main, s:sub
  auto f = [](double x_m, double x_pm, double t_m, double t_pm, double t_s){
    double a = (x_m - x_pm) / (t_m - t_pm);
    double b = x_m - a * t_m;
    return a * t_s + b;
  };

  double x_inp = f(main_pos.x, previous_main_pos.x, t_m, t_pm, t_s);
  double y_inp = f(main_pos.y, previous_main_pos.y, t_m, t_pm, t_s);
  double z_inp = f(main_pos.z, previous_main_pos.z, t_m, t_pm, t_s);

  ggall2fixll(subgga_msg_ptr, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, maingga_msg_ptr_->alt,
    sub_pos.x, sub_pos.y, sub_pos.z, lc_param_);

  double diff_x = sub_pos.x - x_inp;
  double diff_y = sub_pos.y - y_inp;
  double diff_z = sub_pos.z - z_inp;
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

  std::string sensor_frame;
  if(use_change_of_sensor_frame_)
  {
    sensor_frame = sensor_frame_;
  }
  else
  {
    sensor_frame = maingga_msg_ptr_->header.frame_id;
  }

  // get TF sensor to base
  geometry_msgs::TransformStamped::Ptr TF_sensor_to_base_ptr(new geometry_msgs::TransformStamped);
  getTransform(sensor_frame, base_frame_,TF_sensor_to_base_ptr);

  // transform pose_frame to map_frame
  geometry_msgs::PoseStamped::Ptr transformed_pose_msg_ptr(
    new geometry_msgs::PoseStamped);

  transformed_pose_msg_ptr->header = pose_msg.header;
  transformed_pose_msg_ptr->header.frame_id = map_frame_;

  geometry_msgs::TransformStamped TF_map_to_pose;
  TF_map_to_pose.transform.translation.x = pose_msg.pose.position.x;
  TF_map_to_pose.transform.translation.y = pose_msg.pose.position.y;
  TF_map_to_pose.transform.translation.z = pose_msg.pose.position.z;
  TF_map_to_pose.transform.rotation.x = pose_msg.pose.orientation.x;
  TF_map_to_pose.transform.rotation.y = pose_msg.pose.orientation.y;
  TF_map_to_pose.transform.rotation.z = pose_msg.pose.orientation.z;
  TF_map_to_pose.transform.rotation.w = pose_msg.pose.orientation.w;

  TF_sensor_to_base_ptr->transform;
  TF_map_to_pose.transform;
  tf2::Transform TF2_map_to_pose;
  tf2::Transform TF2_sensor_to_base;
  tf2::convert(TF_map_to_pose.transform, TF2_map_to_pose);
  tf2::convert(TF_sensor_to_base_ptr->transform, TF2_sensor_to_base);
  tf2::Transform TF2_map_to_base = TF2_map_to_pose * TF2_sensor_to_base;

  geometry_msgs::Transform TF_sensor_to_base;
  TF_sensor_to_base = tf2::toMsg(TF2_map_to_base);

  transformed_pose_msg_ptr->pose.position.x = TF_sensor_to_base.translation.x;
  transformed_pose_msg_ptr->pose.position.y = TF_sensor_to_base.translation.y;
  transformed_pose_msg_ptr->pose.position.z = TF_sensor_to_base.translation.z;
  transformed_pose_msg_ptr->pose.orientation.x = TF_sensor_to_base.rotation.x;
  transformed_pose_msg_ptr->pose.orientation.y = TF_sensor_to_base.rotation.y;
  transformed_pose_msg_ptr->pose.orientation.z = TF_sensor_to_base.rotation.z;
  transformed_pose_msg_ptr->pose.orientation.w = TF_sensor_to_base.rotation.w;

  publishTF(map_frame_, "gnss_compass_base_link", *transformed_pose_msg_ptr);

  odom_msg_.header = transformed_pose_msg_ptr->header;
  odom_msg_.child_frame_id = "gnss_compass_base_link";
  odom_msg_.pose.pose = transformed_pose_msg_ptr->pose;

  double baseline_length = std::sqrt(pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2));
  bool is_beseline_ok = (beseline_length_ - allowable_beseline_length_error_ <= baseline_length &&
    baseline_length <= beseline_length_ + allowable_beseline_length_error_);
  if(!is_beseline_ok)
  {
    ROS_WARN("mayby mis-FIX:l %lf, yaw,%lf, dt %lf", baseline_length, theta * 180 / M_PI, dt_ms);
    illigal_odom_pub_.publish(odom_msg_);
    return;
  }
  else {
    ROS_INFO("normal       :l %lf, yaw %lf, dt %lf", baseline_length, theta * 180 / M_PI, dt_ms);
  }
  
  pose_pub_.publish(transformed_pose_msg_ptr);
  odom_pub_.publish(odom_msg_);

  geometry_msgs::PoseWithCovarianceStamped transformed_pose_with_covariance_msg;
  transformed_pose_with_covariance_msg.header = transformed_pose_msg_ptr->header;
  transformed_pose_with_covariance_msg.pose.pose = transformed_pose_msg_ptr->pose;

  double std_dev_xyz = 0.03; // [m]
  double std_dev_rp = 100; // [rad]
  double std_dev_yaw = 0.2 / 180 * M_PI; // [rad]
  transformed_pose_with_covariance_msg.pose.covariance[0] = std_dev_xyz * std_dev_xyz;
  transformed_pose_with_covariance_msg.pose.covariance[7] = std_dev_xyz * std_dev_xyz;
  transformed_pose_with_covariance_msg.pose.covariance[14] = std_dev_xyz * std_dev_xyz;
  transformed_pose_with_covariance_msg.pose.covariance[21] = std_dev_rp * std_dev_rp;
  transformed_pose_with_covariance_msg.pose.covariance[28] = std_dev_rp * std_dev_rp;
  transformed_pose_with_covariance_msg.pose.covariance[35] = std_dev_yaw * std_dev_yaw;


  pose_with_covariance_pub_.publish(transformed_pose_with_covariance_msg);

}

void GnssCompass::publishTF(const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

bool GnssCompass::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
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

void GnssCompass::timerDiagnostic()
{
  ros::Rate rate(100);
  while (ros::ok()) {
    key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num_);

    diagnostic_msgs::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "gnss_compass";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= max_skipping_publish_num_) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
      ROS_WARN("Emergency! skipping_publish_num: %d", skipping_publish_num_);
    }

    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_.publish(diag_msg);

    rate.sleep();
  }
}
