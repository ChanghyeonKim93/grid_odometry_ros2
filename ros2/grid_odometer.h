#ifndef GRID_ODOMETER_GRID_ODOMETER_ROS2_H_
#define GRID_ODOMETER_GRID_ODOMETER_ROS2_H_

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "core/grid_odometer.h"

namespace grid_odometer {

class GridOdometerRos2 : public rclcpp::Node {
 public:
  GridOdometerRos2(const std::string& node_name) : Node(node_name) {
    // Load parameters
    Parameters parameters;

    grid_odometer_ = std::make_unique<GridOdometer>(parameters);

    // subscribers
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&GridOdometerRos2::CallbackLaserScan, this,
                      std::placeholders::_1));

    // publishers
    pose_list_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>("~/pose_list", 10);
  }

 private:
  void CallbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    grid_odometer_->Update(ConvertToBridge(*msg));
    // pose_list_.header.frame_id = "map";
    // pose_list_.header.stamp = now();
    // pose_list_.poses.push_back(*msg);

    // pose_list_publisher_->publish(pose_list_);
  }

  bridge::LaserScan ConvertToBridge(const sensor_msgs::msg::LaserScan& msg) {
    bridge::LaserScan bridge_laser_scan;
    bridge_laser_scan.time =
        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    for (size_t index = 0; index < msg.ranges.size(); ++index) {
      const auto range = msg.ranges.at(index);
      if (range < msg.range_min || range > msg.range_max) continue;
      const double point_time =
          msg.time_increment * index + bridge_laser_scan.time;
      const double angle = msg.angle_min + index * msg.angle_increment;
      const double x = std::cos(angle) * range;
      const double y = std::sin(angle) * range;
      bridge::TimedPoint timed_point;
      timed_point.time = point_time;
      timed_point.point.x = x;
      timed_point.point.y = y;
      bridge_laser_scan.data.push_back(timed_point);
    }
    return bridge_laser_scan;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_list_publisher_;

  nav_msgs::msg::Path pose_list_;

 private:
  std::unique_ptr<GridOdometer> grid_odometer_{nullptr};
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_GRID_ODOMETER_ROS2_H_
