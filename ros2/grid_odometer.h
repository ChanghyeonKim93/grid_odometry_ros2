#ifndef GRID_ODOMETER_GRID_ODOMETER_ROS2_H_
#define GRID_ODOMETER_GRID_ODOMETER_ROS2_H_

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

#include "core/grid_odometer.h"

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PathMsg = nav_msgs::msg::Path;
using MarkerMsg = visualization_msgs::msg::Marker;
using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;

namespace grid_odometer {

class GridOdometerRos2 : public rclcpp::Node {
 public:
  GridOdometerRos2(const std::string& node_name) : Node(node_name) {
    // Load parameters
    Parameters parameters;
    parameters.grid_map_updater.resolution = 0.05;

    grid_odometer_ = std::make_unique<GridOdometer>(parameters);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribers
    laser_scan_subscriber_ = this->create_subscription<LaserScanMsg>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&GridOdometerRos2::CallbackLaserScan, this,
                  std::placeholders::_1));

    // Publishers
    pose_list_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "~/pose_list", rclcpp::SensorDataQoS());
    point_cloud_publisher_ = this->create_publisher<PointCloud2Msg>(
        "~/point_cloud", rclcpp::SensorDataQoS());
    raw_point_cloud_publisher_ = this->create_publisher<PointCloud2Msg>(
        "~/raw_point_cloud", rclcpp::SensorDataQoS());
    ray_point_cloud_publisher_ = this->create_publisher<PointCloud2Msg>(
        "~/ray_point_cloud", rclcpp::SensorDataQoS());
    grid_map_publisher_ = this->create_publisher<OccupancyGridMsg>(
        "~/grid_map", rclcpp::SensorDataQoS());
  }

 private:
  void CallbackLaserScan(const LaserScanMsg::SharedPtr msg) {
    const auto bridge_timed_point_cloud = ConvertToBridgeTimedPointCloud(*msg);
    grid_odometer_->Update(bridge_timed_point_cloud);

    ray_point_cloud_publisher_->publish(ConvertToPointCloud2(
        grid_odometer_->GetRayPointList(bridge_timed_point_cloud),
        "base_link"));
    raw_point_cloud_publisher_->publish(
        ConvertToPointCloud2(bridge_timed_point_cloud.data, "base_link"));

    std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;
    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = now();
    stamped_transform.header.frame_id = "map";
    stamped_transform.child_frame_id = "base_link";
    stamped_transform.transform.translation.x = 0.0;
    stamped_transform.transform.translation.y = 0.0;
    stamped_transform.transform.translation.z = 0.0;
    stamped_transform.transform.rotation.w = 1.0;
    stamped_transform.transform.rotation.x = 0.0;
    stamped_transform.transform.rotation.y = 0.0;
    stamped_transform.transform.rotation.z = 0.0;
    stamped_transforms.push_back(stamped_transform);
    tf_broadcaster_->sendTransform(stamped_transforms);

    const auto bridge_grid_map = grid_odometer_->GetGridMap();
    grid_map_publisher_->publish(ConvertToOccupancyGridMsg(bridge_grid_map));
  }

  bridge::TimedPointCloud ConvertToBridgeTimedPointCloud(
      const LaserScanMsg& msg) {
    bridge::TimedPointCloud bridge_timed_point_cloud;
    bridge_timed_point_cloud.time =
        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    for (size_t index = 0; index < msg.ranges.size(); ++index) {
      const auto range = msg.ranges.at(index);
      if (range < msg.range_min || range > msg.range_max) continue;
      const double point_time =
          msg.time_increment * index + bridge_timed_point_cloud.time;
      const double angle = msg.angle_min + index * msg.angle_increment;
      const double x = std::cos(angle) * range;
      const double y = std::sin(angle) * range;
      bridge_timed_point_cloud.data.emplace_back(point_time, x, y);
    }
    return bridge_timed_point_cloud;
  }

  OccupancyGridMsg ConvertToOccupancyGridMsg(
      const bridge::GridMap& bridge_grid_map) {
    OccupancyGridMsg msg;
    msg.info.map_load_time.sec = now().seconds();
    msg.info.map_load_time.nanosec = now().nanoseconds();
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
    msg.info.origin.position.x =
        bridge_grid_map.pose.x +
        bridge_grid_map.origin.x * bridge_grid_map.resolution;
    msg.info.origin.position.y =
        bridge_grid_map.pose.y +
        bridge_grid_map.origin.y * bridge_grid_map.resolution;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = std::cos(bridge_grid_map.pose.yaw);
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = std::sin(bridge_grid_map.pose.yaw);
    msg.info.width = bridge_grid_map.width;
    msg.info.height = bridge_grid_map.height;
    msg.info.resolution = bridge_grid_map.resolution;
    for (const auto& cell : bridge_grid_map.grid_cell_list) {
      msg.data.push_back(cell.hit_count);
    }
    return msg;
  }

  sensor_msgs::msg::PointCloud2 ConvertToPointCloud2(
      const std::vector<bridge::Point>& point_list, std::string frame_id) {
    sensor_msgs::msg::PointCloud2 dst;

    // intensity mapping (-3 m ~ 3 m to 0~255)
    // float z_min = -3.0;
    // float z_max = 3.0;
    // float intensity_min = 30;
    // float intensity_max = 255;
    // float slope = (intensity_max - intensity_min) / (z_max - z_min);

    dst.header.frame_id = frame_id;
    dst.header.stamp = now();
    dst.width = point_list.size();
    dst.height = 1;

    sensor_msgs::msg::PointField point_field;
    point_field.offset = 0;
    point_field.name = "x";
    point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    dst.fields.push_back(point_field);
    point_field.offset = 4;
    point_field.name = "y";
    point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    dst.fields.push_back(point_field);
    point_field.offset = 8;
    point_field.name = "z";
    point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    dst.fields.push_back(point_field);
    // f_tmp.offset = 12;
    // f_tmp.name = "intensity";
    // f_tmp.datatype = sensor_msgs::msg::PointField::FLOAT32;
    // dst.fields.push_back(f_tmp);
    // f_tmp.offset = 16;
    // f_tmp.name = "ring";
    // f_tmp.datatype = sensor_msgs::msg::PointField::UINT16;
    // dst.fields.push_back(f_tmp);
    // f_tmp.offset = 18;
    // f_tmp.name = "time";
    // f_tmp.datatype = sensor_msgs::msg::PointField::FLOAT32;
    // dst.fields.push_back(f_tmp);
    // dst.point_step = 22;  // x 4 + y 4 + z 4 + i 4 + r 2 + t 4
    dst.point_step = 12;  // x 4 y 4 z 4

    dst.data.resize(dst.point_step * dst.width);
    for (size_t i = 0; i < dst.width; ++i) {
      size_t i_ptstep = i * dst.point_step;
      size_t arrayPosX =
          i_ptstep + dst.fields[0].offset;  // X has an offset of 0
      size_t arrayPosY =
          i_ptstep + dst.fields[1].offset;  // Y has an offset of 4
      size_t arrayPosZ =
          i_ptstep + dst.fields[2].offset;  // Z has an offset of 8

      // size_t ind_intensity = i_ptstep + dst.fields[3].offset;  // 12
      // size_t ind_ring = i_ptstep + dst.fields[4].offset;       // 16
      // size_t ind_time = i_ptstep + dst.fields[5].offset;       // 18

      // float height_intensity = slope * (X[i](2) - z_min) + intensity_min;
      // if (height_intensity >= intensity_max) height_intensity =
      // intensity_max; if (height_intensity <= intensity_min) height_intensity
      // = intensity_min;
      float x = point_list.at(i).x;
      float y = point_list.at(i).y;
      float z = 0.0f;

      memcpy(&dst.data[arrayPosX], &(x), sizeof(float));
      memcpy(&dst.data[arrayPosY], &(y), sizeof(float));
      memcpy(&dst.data[arrayPosZ], &(z), sizeof(float));
      // memcpy(&dst.data[ind_intensity], &(height_intensity), sizeof(float));
      // memcpy(&dst.data[ind_ring], &(x), sizeof(unsigned short));
      // memcpy(&dst.data[ind_time], &(x), sizeof(float));
    }
    return dst;
  }

  sensor_msgs::msg::PointCloud2 ConvertToPointCloud2(
      const std::vector<bridge::TimedPoint>& timed_point_list,
      std::string frame_id) {
    sensor_msgs::msg::PointCloud2 dst;

    // intensity mapping (-3 m ~ 3 m to 0~255)
    // float z_min = -3.0;
    // float z_max = 3.0;
    // float intensity_min = 30;
    // float intensity_max = 255;
    // float slope = (intensity_max - intensity_min) / (z_max - z_min);

    dst.header.frame_id = frame_id;
    dst.header.stamp = now();
    dst.width = timed_point_list.size();
    dst.height = 1;

    sensor_msgs::msg::PointField point_field;
    point_field.offset = 0;
    point_field.name = "x";
    point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    dst.fields.push_back(point_field);
    point_field.offset = 4;
    point_field.name = "y";
    point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    dst.fields.push_back(point_field);
    point_field.offset = 8;
    point_field.name = "z";
    point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    dst.fields.push_back(point_field);
    // f_tmp.offset = 12;
    // f_tmp.name = "intensity";
    // f_tmp.datatype = sensor_msgs::msg::PointField::FLOAT32;
    // dst.fields.push_back(f_tmp);
    // f_tmp.offset = 16;
    // f_tmp.name = "ring";
    // f_tmp.datatype = sensor_msgs::msg::PointField::UINT16;
    // dst.fields.push_back(f_tmp);
    // f_tmp.offset = 18;
    // f_tmp.name = "time";
    // f_tmp.datatype = sensor_msgs::msg::PointField::FLOAT32;
    // dst.fields.push_back(f_tmp);
    // dst.point_step = 22;  // x 4 + y 4 + z 4 + i 4 + r 2 + t 4
    dst.point_step = 12;  // x 4 y 4 z 4

    dst.data.resize(dst.point_step * dst.width);
    for (size_t i = 0; i < dst.width; ++i) {
      size_t i_ptstep = i * dst.point_step;
      size_t arrayPosX =
          i_ptstep + dst.fields[0].offset;  // X has an offset of 0
      size_t arrayPosY =
          i_ptstep + dst.fields[1].offset;  // Y has an offset of 4
      size_t arrayPosZ =
          i_ptstep + dst.fields[2].offset;  // Z has an offset of 8

      // size_t ind_intensity = i_ptstep + dst.fields[3].offset;  // 12
      // size_t ind_ring = i_ptstep + dst.fields[4].offset;       // 16
      // size_t ind_time = i_ptstep + dst.fields[5].offset;       // 18

      // float height_intensity = slope * (X[i](2) - z_min) + intensity_min;
      // if (height_intensity >= intensity_max) height_intensity =
      // intensity_max; if (height_intensity <= intensity_min) height_intensity
      // = intensity_min;
      float x = timed_point_list.at(i).point.x;
      float y = timed_point_list.at(i).point.y;
      float z = 0.0f;

      memcpy(&dst.data[arrayPosX], &(x), sizeof(float));
      memcpy(&dst.data[arrayPosY], &(y), sizeof(float));
      memcpy(&dst.data[arrayPosZ], &(z), sizeof(float));
      // memcpy(&dst.data[ind_intensity], &(height_intensity), sizeof(float));
      // memcpy(&dst.data[ind_ring], &(x), sizeof(unsigned short));
      // memcpy(&dst.data[ind_time], &(x), sizeof(float));
    }
    return dst;
  }

 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<LaserScanMsg>::SharedPtr laser_scan_subscriber_;

  rclcpp::Publisher<PointCloud2Msg>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<PointCloud2Msg>::SharedPtr raw_point_cloud_publisher_;
  rclcpp::Publisher<PointCloud2Msg>::SharedPtr ray_point_cloud_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_list_publisher_;

  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr grid_map_publisher_;

  nav_msgs::msg::Path pose_list_;

 private:
  std::unique_ptr<GridOdometer> grid_odometer_{nullptr};
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_GRID_ODOMETER_ROS2_H_
