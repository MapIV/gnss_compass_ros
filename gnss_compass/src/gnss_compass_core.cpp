#include <gnss_compass_core/gnss_compass_core.hpp>

GnssCompass::GnssCompass():Node("gnss_compass")
{

  auto node = rclcpp::Node::make_shared("gnss_compass");

  node->declare_parameter("map_frame",map_frame_);
  node->declare_parameter("base_frame",base_frame_);
  node->declare_parameter("use_mgrs",use_mgrs_);
  node->declare_parameter("plane_num",plane_num_);
  node->declare_parameter("use_change_of_sensor_frame",use_change_of_sensor_frame_);
  node->declare_parameter("sensor_frame",sensor_frame_);
  node->declare_parameter("gnss_frequency",gnss_frequency_);
  node->declare_parameter("min_gnss_status",min_gnss_status_);
  node->declare_parameter("max_gnss_status",max_gnss_status_);
  node->declare_parameter("time_threshold",time_thresshold_);
  node->declare_parameter("yaw_bias",yaw_bias_);
  node->declare_parameter("use_simple_roswarn",use_simple_roswarn_);
  node->declare_parameter("baseline_length",beseline_length_);
  node->declare_parameter("allowable_baseline_length_error",allowable_beseline_length_error_);
  node->declare_parameter("max_skipping_publish_num",max_skipping_publish_num_);

  node->get_parameter("map_frame",map_frame_);
  node->get_parameter("base_frame",base_frame_);
  node->get_parameter("use_mgrs",use_mgrs_);
  node->get_parameter("plane_num",plane_num_);
  node->get_parameter("use_change_of_sensor_frame",use_change_of_sensor_frame_);
  node->get_parameter("sensor_frame",sensor_frame_);
  node->get_parameter("gnss_frequency",gnss_frequency_);
  node->get_parameter("min_gnss_status",min_gnss_status_);
  node->get_parameter("max_gnss_status",max_gnss_status_);
  node->get_parameter("time_threshold",time_thresshold_);
  node->get_parameter("yaw_bias",yaw_bias_);
  node->get_parameter("use_simple_roswarn",use_simple_roswarn_);
  node->get_parameter("baseline_length",beseline_length_);
  node->get_parameter("allowable_baseline_length_error",allowable_beseline_length_error_);
  node->get_parameter("max_skipping_publish_num",max_skipping_publish_num_);

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
  RCLCPP_INFO(get_logger(),"min_gnss_status: %d, max_gnss_status: %d, time_thresshold: %lf, yaw_bias: %lf", min_gnss_status_,
    max_gnss_status_, time_thresshold_, yaw_bias_);
  RCLCPP_INFO(get_logger(),"use_simple_roswarn: %d, beseline_length: %lf, allowable_beseline_length_error: %lf, max_skipping_publish_num: %d", use_simple_roswarn_,
    beseline_length_, allowable_beseline_length_error_, max_skipping_publish_num_);

  maingga_sub_ = this->create_subscription<nmea_msgs::msg::Gpgga>("main_gnss_gga", 100, std::bind(&GnssCompass::callbackMainGga, this,std::placeholders::_1));
  subgga_sub_ = this->create_subscription<nmea_msgs::msg::Gpgga>("sub_gnss_gga", 100, std::bind(&GnssCompass::callbackSubGga, this,std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gnss_compass_pose", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("gnss_compass_odom", 10);
  illigal_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("illigal_gnss_compass_odom", 10);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

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


double GnssCompass::toSec(const std_msgs::msg::Header &msg){
  return msg.stamp.sec + msg.stamp.nanosec/1e9;
}

double GnssCompass::calcYaw(const xyzt & main_pos, const xyzt & previous_main_pos, const xyzt & sub_pos, double & baseline_length)
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


void GnssCompass::callbackMainGga(const nmea_msgs::msg::Gpgga::ConstSharedPtr  maingga_msg_ptr)
{
  int gps_qual = maingga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  if(!is_gnss_status_ok)
  {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Main is not fixed. status is %d", maingga_msg_ptr->gps_qual);
    return;
  }
  if(maingga_msg_ptr != nullptr)
  {
    previous_maingga_msg_ptr_ = maingga_msg_ptr_;
  }
  maingga_msg_ptr_ = maingga_msg_ptr;

}

void GnssCompass::callbackSubGga(const nmea_msgs::msg::Gpgga::ConstSharedPtr  subgga_msg_ptr)
{
  if (maingga_msg_ptr_ == nullptr || previous_maingga_msg_ptr_ == nullptr) {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Main is not subscrubbed.");
    skipping_publish_num_++;
    return;
  }
  double t_pm = toSec(previous_maingga_msg_ptr_->header);
  double t_m = toSec(maingga_msg_ptr_->header);
  double dt_mm = t_pm - t_m;
  if(std::abs(dt_mm) > 1.5 * 1.0 / gnss_frequency_) { // Up to 150% allowed
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"The difference between timestamps is large:dt_mm %lf.", dt_mm);
    skipping_publish_num_++;
    return;
  }

  double t_s = toSec(subgga_msg_ptr->header);
  double dt_ms = t_s - t_m;
  if(std::abs(dt_ms) > time_thresshold_ || dt_ms < 0) {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"The difference between timestamps is large:dt_ms %lf.", dt_ms);
    skipping_publish_num_++;
    return;
  }

  int gps_qual = subgga_msg_ptr->gps_qual;
  bool is_gnss_status_ok = (min_gnss_status_ <= gps_qual && gps_qual <= max_gnss_status_);
  if(!is_gnss_status_ok)
  {
    if(!use_simple_roswarn_) RCLCPP_WARN(get_logger(),"Sub is not fixed %d", subgga_msg_ptr->gps_qual);
    skipping_publish_num_++;
    return;
  }

  skipping_publish_num_ = 0;

  xyzt main_pos, previous_main_pos, sub_pos;
  double navsat_lat, navsat_lon, previous_navsat_lat, previous_navsat_lon;

  ggall2fixll(maingga_msg_ptr_, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, maingga_msg_ptr_->alt,
    main_pos.x, main_pos.y, main_pos.z, lc_param_);
  main_pos.time = t_m;

  ggall2fixll(previous_maingga_msg_ptr_, previous_navsat_lat, previous_navsat_lon);
  llh_converter_.convertDeg2XYZ(previous_navsat_lat, previous_navsat_lon, previous_maingga_msg_ptr_->alt,
    previous_main_pos.x, previous_main_pos.y, previous_main_pos.z, lc_param_);
  previous_main_pos.time = t_pm;

  ggall2fixll(subgga_msg_ptr, navsat_lat, navsat_lon);
  llh_converter_.convertDeg2XYZ(navsat_lat, navsat_lon, maingga_msg_ptr_->alt,
    sub_pos.x, sub_pos.y, sub_pos.z, lc_param_);
  sub_pos.time = t_s;

  double baseline_length;
  double theta  = calcYaw(main_pos, previous_main_pos, sub_pos, baseline_length);

  tf2::Quaternion localization_quat;
  localization_quat.setRPY(0, 0, theta);
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(localization_quat);

  geometry_msgs::msg::PoseStamped pose_msg;
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
  if(!is_beseline_ok)
  {
    RCLCPP_WARN(get_logger(),"mayby mis-FIX:l %lf, yaw,%lf, dt %lf", baseline_length, theta * 180 / M_PI, dt_ms);
    illigal_odom_pub_->publish(odom_msg_);
    return;
  }
  RCLCPP_ERROR(get_logger(),"normal       :l %lf, yaw %lf, dt %lf", baseline_length, theta * 180 / M_PI, dt_ms);

  pose_pub_->publish(*transformed_pose_msg_ptr);
  odom_pub_->publish(odom_msg_);

}

void GnssCompass::publishTF(const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::PoseStamped & pose_msg)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
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
  tf2_broadcaster_->sendTransform(transform_stamped);
}

bool GnssCompass::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr)
{

  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = this->get_clock()->now();
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
      tf2_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(),"%s", ex.what());
    RCLCPP_ERROR(get_logger(),"Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = this->get_clock()->now();
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

void GnssCompass::ggall2fixll(const nmea_msgs::msg::Gpgga::ConstSharedPtr & gga_msg_ptr, double & navsat_lat, double & navsat_lon)
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
      RCLCPP_WARN(get_logger(),"Emergency! skipping_publish_num: %d", skipping_publish_num_);
    }

    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->get_clock()->now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_->publish(diag_msg);

    rate.sleep();
  }
}
