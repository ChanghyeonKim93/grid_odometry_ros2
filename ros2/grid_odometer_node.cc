#include <iostream>
#include <stdexcept>
#include <string>

#include "ros2/grid_odometer.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    std::string node_name = "grid_odometer_node";
    rclcpp::spin(std::make_shared<grid_odometer::GridOdometerRos2>(node_name));
    rclcpp::shutdown();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
