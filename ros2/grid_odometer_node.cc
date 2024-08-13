#include <iostream>
#include <stdexcept>
#include <string>

#include "ros2/grid_odometer.h"

const std::string node_name{"grid_odometer_node"};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<grid_odometer::GridOdometerRos2>(node_name));
    rclcpp::shutdown();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
