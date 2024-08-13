#include "core/grid_odometer.h"

#include <iostream>

namespace grid_odometer {

GridOdometer::GridOdometer(const Parameters& parameters)
    : parameters_{parameters} {
  grid_map_updater_ = std::make_unique<grid_map_updater::GridMapUpdater>(
      parameters_.grid_map_updater);
}

void GridOdometer::Update(
    const bridge::TimedPointCloud& bridge_timed_point_cloud) {
  const auto timed_point_list =
      ConvertToTimedPointList(bridge_timed_point_cloud.data);
  if (!state_.is_initialized) {
    state_.is_initialized = InitializeGridMap(timed_point_list);
    return;
  }

  // Estimate current pose

  // Update grid map
  grid_map_updater_->UpdateGridMap(
      Pose::Identity(), ConvertToPointList(bridge_timed_point_cloud.data),
      &current_grid_map_);

  // If motion is large, update cells by current points
}

bridge::GridMap GridOdometer::GetGridMap() const {
  bridge::GridMap bridge_grid_map;
  bridge_grid_map.id = current_grid_map_.id;
  bridge_grid_map.resolution = current_grid_map_.resolution;
  bridge_grid_map.height = current_grid_map_.height;
  bridge_grid_map.width = current_grid_map_.width;
  bridge_grid_map.origin.x = current_grid_map_.origin.x();
  bridge_grid_map.origin.y = current_grid_map_.origin.y();
  bridge_grid_map.grid_cell_list.reserve(current_grid_map_.cell_list.size());
  for (const auto& grid_cell : current_grid_map_.cell_list) {
    bridge::GridCell bridge_grid_cell;
    bridge_grid_cell.hit_count = grid_cell.hit_count;
    bridge_grid_map.grid_cell_list.push_back(bridge_grid_cell);
  }
  return bridge_grid_map;
}

std::vector<bridge::Point> GridOdometer::GetRayPointList(
    const bridge::TimedPointCloud& bridge_laser_scan) {
  const auto point_list = ConvertToPointList(bridge_laser_scan.data);

  std::vector<bridge::Point> ray_point_list;
  for (const auto& point : point_list) {
    const auto ray_point_list_for_point =
        grid_map_updater_->GenerateRayPointList(Vec2::Zero(), point);
    for (const auto& ray_point : ray_point_list_for_point)
      ray_point_list.emplace_back(ray_point.x(), ray_point.y());
  }

  return ray_point_list;
}

bool GridOdometer::InitializeGridMap(
    const std::vector<grid_odometer::TimedPoint>& timed_point_list) {
  // Add initial pose
  Pose initial_pose{Pose::Identity()};
  pose_list_.push_back(initial_pose);
  previous_pose_.setIdentity();

  std::vector<Point> point_list;
  for (const auto& timed_point : timed_point_list)
    point_list.push_back(timed_point.point);

  grid_map_updater_->UpdateGridMap(Pose::Identity(), point_list,
                                   &current_grid_map_);

  return true;
}

Pose GridOdometer::EstimatedPose(
    const Pose& initial_pose, const std::vector<TimedPoint>& timed_point_list) {
  Pose estimated_pose = initial_pose;

  // Find nearest neighbor
  // Estimated pose

  return estimated_pose;
}

std::vector<TimedPoint> GridOdometer::ConvertToTimedPointList(
    const std::vector<bridge::TimedPoint>& bridge_timed_point_list) {
  std::vector<TimedPoint> timed_point_list;
  timed_point_list.reserve(bridge_timed_point_list.size());
  for (const auto& bridge_timed_point : bridge_timed_point_list) {
    TimedPoint timed_point;
    timed_point.time = bridge_timed_point.time;
    timed_point.point.x() = bridge_timed_point.point.x;
    timed_point.point.y() = bridge_timed_point.point.y;
    timed_point_list.push_back(timed_point);
  }
  return timed_point_list;
}

std::vector<Point> GridOdometer::ConvertToPointList(
    const std::vector<bridge::TimedPoint>& bridge_timed_point_list) {
  std::vector<Point> point_list;
  point_list.reserve(bridge_timed_point_list.size());
  for (const auto& bridge_timed_point : bridge_timed_point_list)
    point_list.emplace_back(bridge_timed_point.point.x,
                            bridge_timed_point.point.y);
  return point_list;
}

}  // namespace grid_odometer