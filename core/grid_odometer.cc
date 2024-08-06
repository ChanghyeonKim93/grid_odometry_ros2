#include "core/grid_odometer.h"

#include <iostream>

namespace grid_odometer {

GridOdometer::GridOdometer(const Parameters& parameters)
    : parameters_{parameters} {}

void GridOdometer::Update(const bridge::LaserScan& bridge_laser_scan) {
  for (const auto& bridge_timed_point : bridge_laser_scan.data) {
    TimedPoint timed_point;
    timed_point.time = bridge_timed_point.time;
    timed_point.point.x() = bridge_timed_point.point.x;
    timed_point.point.y() = bridge_timed_point.point.y;
    const auto& key = ComputeGridKey(timed_point.point);
    if (grid_map_.grid_cell_list.find(key) == grid_map_.grid_cell_list.end())
      grid_map_.grid_cell_list.insert({key, {}});
    else {
      auto& point_list = grid_map_.grid_cell_list.at(key).point_list;
      auto& mean = grid_map_.grid_cell_list.at(key).mean;
      point_list.push_back(timed_point.point);
      if (point_list.size() > 20) point_list.pop_front();
      Eigen::Vector2d sum{Eigen::Vector2d::Zero()};
      for (const auto& point : point_list) sum += point;
      mean = sum / point_list.size();
    }
  }
}

bridge::GridMap GridOdometer::GetGridMap() const {
  bridge::GridMap bridge_grid_map;
  bridge_grid_map.id = grid_map_.id;
  bridge_grid_map.grid_cell_list.reserve(grid_map_.grid_cell_list.size());
  for (const auto& [key, grid_cell] : grid_map_.grid_cell_list) {
    bridge::GridCell bridge_grid_cell;
    bridge_grid_cell.mean.x = grid_cell.mean.x();
    bridge_grid_cell.mean.y = grid_cell.mean.y();
    bridge_grid_map.grid_cell_list.insert({key, bridge_grid_cell});
  }
  return bridge_grid_map;
}

uint64_t GridOdometer::ComputeGridKey(const Point point) {
  static double inverse_grid_resolution = 1.0 / parameters_.grid_resolution;
  const int x_index =
      static_cast<int>(std::round(point.x() * inverse_grid_resolution));
  const int y_index =
      static_cast<int>(std::round(point.y() * inverse_grid_resolution));
  const uint32_t x_key = (x_index > 0) ? 2 * x_index : -2 * x_index + 1;
  const uint32_t y_key = (y_index > 0) ? 2 * y_index : -2 * y_index + 1;
  const uint64_t key = (x_key + y_key) * (x_key + y_key + 1) + y_key;
  return key;
}

}  // namespace grid_odometer