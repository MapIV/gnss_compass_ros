#include <gnss_compass_core/gnss_compass_core.hpp>

GnssCompass::GnssCompass():Node("gnss_compass")
{
  int input_gnss_type;
  bool use_mgrs;
  int plane_num;

  this->declare_parameter("map_frame",map_frame_);
  this->declare_parameter("base_frame",base_frame_);
  this->declare_parameter("use_mgrs",use_mgrs);
  this->declare_parameter("plane_num",plane_num);
  this->declare_parameter("use_change_of_sensor_frame",use_change_of_sensor_frame_);
  this->declare_parameter("sensor_frame",sensor_frame_);
  this->declare_parameter("gnss_frequency",gnss_frequency_);
  this->declare_parameter("input_gnss_type",input_gnss_type);
  this->declare_parameter("min_gnss_status",min_gnss_status_);
  this->declare_parameter("max_gnss_status",max_gnss_status_);
  this->declare_parameter("time_threshold",time_thresshold_);
  this->declare_parameter("yaw_bias",yaw_bias_);
  this->declare_parameter("use_simple_roswarn",use_simple_roswarn_);
  this->declare_parameter("use_beseline_outlier_detection",use_beseline_outlier_detection_);
  this->declare_parameter("baseline_length",beseline_length_);
  this->declare_parameter("allowable_baseline_length_error",allowable_beseline_length_error_);
  this->declare_parameter("max_skipping_publish_num",max_skipping_publish_num_);

  this->get_parameter("map_frame",map_frame_);
  this->get_parameter("base_frame",base_frame_);
  this->get_parameter("use_mgrs",use_mgrs);
  this->get_parameter("plane_num",plane_num);
  this->get_parameter("use_change_of_sensor_frame",use_change_of_sensor_frame_);
  this->get_parameter("sensor_frame",sensor_frame_);
  this->get_parameter("gnss_frequency",gnss_frequency_);
  this->get_parameter("input_gnss_type",input_gnss_type);
  this->get_parameter("min_gnss_status",min_gnss_status_);
  this->get_parameter("max_gnss_status",max_gnss_status_);
  this->get_parameter("fix_covariance_thershold",fix_covariance_thershold_);
  this->get_parameter("time_threshold",time_thresshold_);
  this->get_parameter("yaw_bias",yaw_bias_);
  this->get_parameter("use_simple_roswarn",use_simple_roswarn_);
  this->get_parameter("use_beseline_outlier_detection",use_beseline_outlier_detection_);
  this->get_parameter("baseline_length",beseline_length_);
  this->get_parameter("allowable_baseline_length_error",allowable_beseline_length_error_);
  this->get_parameter("max_skipping_publish_num",max_skipping_publish_num_);

  tf2_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  tf2_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_.c_str());
  if(use_change_of_sensor_frame_)
  {
    RCLCPP_INFO(get_logger(),"sensor_frame_id: %s", sensor_frame_.c_str());
  }
  RCLCPP_INFO(get_logger(),"min_gnss_status: %d, max_gnss_status: %d, fix_covariance_thershold: %f, time_thresshold: %lf, yaw_bias: %lf", min_gnss_status_,
    max_gnss_status_, fix_covariance_thershold_, time_thresshold_, yaw_bias_);
  RCLCPP_INFO(get_logger(),"use_simple_roswarn: %d, beseline_length: %lf, allowable_beseline_length_error: %lf, max_skipping_publish_num: %d", use_simple_roswarn_,
    beseline_length_, allowable_beseline_length_error_, max_skipping_publish_num_);

  if(input_gnss_type == 0)
  {
    maingga_sub_ = this->create_subscription<nmea_msgs::msg::Gpgga>("main_gnss_gga", 100, std::bind(&GnssCompass::callbackMainGga, this,std::placeholders::_1));
    subgga_sub_ = this->create_subscription<nmea_msgs::msg::Gpgga>("sub_gnss_gga", 100, std::bind(&GnssCompass::callbackSubGga, this,std::placeholders::_1));
  }
  else if(input_gnss_type == 1)
  {
    mainfix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("main_gnss_fix", 100, std::bind(&GnssCompass::callbackMainFix, this,std::placeholders::_1));
    subfix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("sub_gnss_fix", 100, std::bind(&GnssCompass::callbackSubFix, this,std::placeholders::_1));
    std::cout << "aa" << std::endl;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "input_gnss_type is not valid");
    rclcpp::shutdown();
  }
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gnss_compass_pose", 10);
  pose_with_covariance_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gnss_compass_pose_with_covariance", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("gnss_compass_odom", 10);
  illigal_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("illigal_gnss_compass_odom", 10);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  diagnostic_thread_ = std::thread(&GnssCompass::timerDiagnostic, this);
  diagnostic_thread_.detach();

  // LLHConverter setting
  lc_param_.use_mgrs = use_mgrs;
  lc_param_.plane_num = plane_num;
  lc_param_.height_convert_type = llh_converter::ConvertType::NONE;
  lc_param_.geoid_type = llh_converter::GeoidType::EGM2008;

  skipping_publish_num_ = 0;
}

GnssCompass::~GnssCompass() {}

double GnssCompass::toSec(const std_msgs::msg::Header &msg){
  return msg.stamp.sec + msg.stamp.nanosec/1e9;
}

void GnssCompass::callbackMainFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr  mainfix_msg_ptr)
{
  xyzts main_pos;
  double fix_covariance = mainfix_msg_ptr->position_covariance[0] ;
  bool is_gnss_status_ok = (fix_covariance <= fix_covariance_thershold_);
  if(!is_gnss_status_ok)
  {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Main fix covariance is too large. position_covariance[0] is %f", fix_covariance);
    main_pos.status = false;
    return;
  }
  else main_pos.status = true;

  main_antenna_header_ = mainfix_msg_ptr->header;

  double navsat_lat, navsat_lon;
  llh_converter_.convertDeg2XYZ(mainfix_msg_ptr->latitude, mainfix_msg_ptr->longitude, mainfix_msg_ptr->altitude,
    main_pos.x, main_pos.y, main_pos.z, lc_param_);
  main_pos.time = toSec(mainfix_msg_ptr->header);

  if(mainfix_msg_ptr != nullptr)
  {
    previous_main_pos_ = main_pos_;
  }
  main_pos_ = main_pos;

}

void GnssCompass::callbackSubFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr  subfix_msg_ptr)
{

  xyzts sub_pos;
  double navsat_lat, navsat_lon, previous_navsat_lat, previous_navsat_lon;

  llh_converter_.convertDeg2XYZ(subfix_msg_ptr->latitude, subfix_msg_ptr->longitude, subfix_msg_ptr->altitude,
    sub_pos.x, sub_pos.y, sub_pos.z, lc_param_);
  double fix_covariance = subfix_msg_ptr->position_covariance[0] ;
  bool is_gnss_status_ok = (fix_covariance <= fix_covariance_thershold_);
  sub_pos.status = is_gnss_status_ok;
  sub_pos.time = toSec(subfix_msg_ptr->header);

  processGnss(main_pos_, previous_main_pos_, sub_pos, main_antenna_header_);

}

void GnssCompass::callbackMainGga(const nmea_msgs::msg::Gpgga::ConstSharedPtr  maingga_msg_ptr)
{
  xyzts main_pos;
  int gps_qual = maingga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  if(!is_gnss_status_ok)
  {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Main is not fixed. status is %d", maingga_msg_ptr->gps_qual);
    main_pos.status = false;
    return;
  }
  else
  {
    main_pos.status = true;
  }

  main_antenna_header_ = maingga_msg_ptr->header;

  double navsat_lat, navsat_lon;
  ggall2fixll(maingga_msg_ptr, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, maingga_msg_ptr->alt,
    main_pos.x, main_pos.y, main_pos.z, lc_param_);
  main_pos.time = toSec(maingga_msg_ptr->header);

  if(maingga_msg_ptr != nullptr)
  {
    previous_main_pos_ = main_pos_;
  }

  main_pos_ = main_pos;

}

void GnssCompass::callbackSubGga(const nmea_msgs::msg::Gpgga::ConstSharedPtr subgga_msg_ptr)
{
  xyzts main_pos, previous_main_pos, sub_pos;
  double navsat_lat, navsat_lon, previous_navsat_lat, previous_navsat_lon;

  ggall2fixll(subgga_msg_ptr, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, subgga_msg_ptr->alt,
    sub_pos.x, sub_pos.y, sub_pos.z, lc_param_);
  sub_pos.time = toSec(subgga_msg_ptr->header);
  int gps_qual = subgga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  sub_pos.status = is_gnss_status_ok;

  processGnss(main_pos_, previous_main_pos_, sub_pos, main_antenna_header_);

}

double GnssCompass::calcYaw(const xyzts & main_pos, const xyzts & previous_main_pos, const xyzts & sub_pos, double & baseline_length)
{
  double t_m = main_pos.time;
  double t_pm = previous_main_pos.time;
  double t_s = sub_pos.time;
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

  double diff_x = sub_pos.x - x_inp;
  double diff_y = sub_pos.y - y_inp;
  double diff_z = sub_pos.z - z_inp;
  baseline_length = std::sqrt(pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2));

  double theta = - std::atan2(diff_x, diff_y) + yaw_bias_;
  return theta;
}

void GnssCompass::processGnss(const xyzts & main_pos, const xyzts & previous_main_pos, const xyzts & sub_pos, std_msgs::msg::Header main_antenna_header)
{

  double dt_mm = previous_main_pos.time - main_pos.time;
  double dt_ms = sub_pos.time - main_pos.time;
  if(std::abs(dt_mm) > 1.5 * 1.0 / gnss_frequency_) { // Up to 150% allowed
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"The difference between timestamps is large:dt_mm %lf.", dt_mm);
    skipping_publish_num_++;
    return;
  }
  if(std::abs(dt_ms) > time_thresshold_ || dt_ms < 0) {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"The difference between timestamps is large:dt_ms %lf.", dt_ms);
    skipping_publish_num_++;
    return;
  }

  if (!main_pos.status || !previous_main_pos.status || dt_mm == 0) {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Poor performance of Main GNSS positioning solutions");
    skipping_publish_num_++;
    return;
  }

  if(!sub_pos.status)
  {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Poor performance of Sub GNSS positioning solutions");
    skipping_publish_num_++;
    return;
  }

  skipping_publish_num_ = 0;

  double baseline_length;
  double theta  = calcYaw(main_pos, previous_main_pos, sub_pos, baseline_length);

  tf2::Quaternion localization_quat;
  localization_quat.setRPY(0, 0, theta);
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(localization_quat);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = main_antenna_header;
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
    sensor_frame = main_antenna_header.frame_id;
  }

  // get TF sensor to base
  geometry_msgs::msg::TransformStamped::SharedPtr TF_sensor_to_base_ptr(new geometry_msgs::msg::TransformStamped);

  getTransform(sensor_frame, base_frame_,TF_sensor_to_base_ptr);

  // transform pose_frame to map_frame
  geometry_msgs::msg::PoseStamped::SharedPtr transformed_pose_msg_ptr(
    new geometry_msgs::msg::PoseStamped);

  transformed_pose_msg_ptr->header.frame_id = pose_msg.header.frame_id;
  transformed_pose_msg_ptr->header.stamp = pose_msg.header.stamp;
  transformed_pose_msg_ptr->header.frame_id = map_frame_;

  geometry_msgs::msg::TransformStamped TF_map_to_pose;
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

  geometry_msgs::msg::Transform TF_sensor_to_base;
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

  bool is_beseline_ok = (beseline_length_ - allowable_beseline_length_error_ <= baseline_length &&
    baseline_length <= beseline_length_ + allowable_beseline_length_error_);
  if(use_beseline_outlier_detection_)
  {
    if(!is_beseline_ok)
    {
      RCLCPP_WARN(get_logger(),"outlier FIX:l %lf, yaw,%lf", baseline_length, theta * 180 / M_PI);
      illigal_odom_pub_->publish(odom_msg_);
      return;
    }
    if(!use_simple_roswarn_) RCLCPP_INFO(get_logger(),"normal       :l %lf, yaw %lf", baseline_length, theta * 180 / M_PI);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
  pose_with_covariance.header = transformed_pose_msg_ptr->header;
  pose_with_covariance.pose.pose = transformed_pose_msg_ptr->pose;
  // TODO(Map IV): temporary value
  double std_dev_roll = 100; // [rad]
  double std_dev_pitch = 100; // [rad]
  double std_dev_yaw = std::atan2(0.05, baseline_length);
  pose_with_covariance.pose.covariance[0] = 0.01;
  pose_with_covariance.pose.covariance[7] = 0.01;
  pose_with_covariance.pose.covariance[14] = 0.04;
  pose_with_covariance.pose.covariance[21] = std_dev_roll * std_dev_roll;
  pose_with_covariance.pose.covariance[28] = std_dev_pitch * std_dev_pitch;
  pose_with_covariance.pose.covariance[35] = std_dev_yaw * std_dev_yaw;

  pose_pub_->publish(*transformed_pose_msg_ptr);
  pose_with_covariance_pub_->publish(pose_with_covariance);
  odom_pub_->publish(odom_msg_);
 return;
}

void GnssCompass::timerDiagnostic()
{
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num_);

    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "gnss_compass";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::msg::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > max_skipping_publish_num_) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
      if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Emergency! skipping_publish_num: %d", skipping_publish_num_);
    }

    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->get_clock()->now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_->publish(diag_msg);

    rate.sleep();
  }
}

