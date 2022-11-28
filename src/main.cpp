#include "rclcpp/rclcpp.hpp"

#include "agrorob_visualization/agrorob_state_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Agrorob visualization started");
  rclcpp::spin(std::make_shared<agrorob_visualization::AgrorobStatePublisher>());
  rclcpp::shutdown();
  return 0;
}