#include <gnss_compass_core/gnss_compass_core.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssCompass>());
  rclcpp::shutdown();

  return 0;
}
