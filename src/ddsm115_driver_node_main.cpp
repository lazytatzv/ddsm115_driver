#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ddsm115_ros2_driver/ddsm115_driver_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ddsm115_ros2_driver::DDSM115DriverNode>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
