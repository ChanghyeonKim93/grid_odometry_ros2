#include "core/grid_odometer.h"

#include <iostream>

namespace grid_odometer {

GridOdometer::GridOdometer(const Parameters& parameters)
    : parameters_{parameters} {}

void GridOdometer::Update(const bridge::LaserScan& bridge_laser_scan) {
  const auto timed_point_list = ConvertToTimedPointList(bridge_laser_scan.data);
  if (!state_.is_initialized) {
    state_.is_initialized = InitializeGridMap(timed_point_list);
    return;
  }

  // Estimate current pose

  // If motion is large, update cells by current points

  const size_t current_pose_index = pose_list_.size() - 1;
  for (const auto& timed_point : timed_point_list) {
    const auto& key = ComputeGridKey(timed_point.point);
    if (grid_map_.cell_list.find(key) == grid_map_.cell_list.end())
      grid_map_.cell_list.insert({key, {}});
    else {
      auto& point_list = grid_map_.cell_list.at(key).point_list_per_pose.at(
          current_pose_index);
      point_list.push_back(timed_point.point);
      // if (point_list.size() > 20) point_list.pop_front();
    }
  }
}

bridge::GridMap GridOdometer::GetGridMap() const {
  bridge::GridMap bridge_grid_map;
  bridge_grid_map.id = grid_map_.id;
  bridge_grid_map.grid_cell_list.reserve(grid_map_.cell_list.size());
  for (const auto& [key, grid_cell] : grid_map_.cell_list) {
    bridge::GridCell bridge_grid_cell;
    bridge_grid_cell.mean.x = grid_cell.mean.x();
    bridge_grid_cell.mean.y = grid_cell.mean.y();
    bridge_grid_map.grid_cell_list.insert({key, bridge_grid_cell});
  }
  return bridge_grid_map;
}

uint64_t GridOdometer::ComputeGridKey(const Point point) {
  static double inverse_grid_resolution = 1.0 / parameters_.grid_resolution;
  const int64_t xi =
      static_cast<int64_t>(std::round(point.x() * inverse_grid_resolution));
  const int64_t yi =
      static_cast<int64_t>(std::round(point.y() * inverse_grid_resolution));
  const uint64_t x_key = (xi > 0) ? 2 * xi : -2 * xi + 1;
  const uint64_t y_key = (yi > 0) ? 2 * yi : -2 * yi + 1;
  const uint64_t key = (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key;
  return key;
}

bool GridOdometer::InitializeGridMap(
    const std::vector<grid_odometer::TimedPoint>& timed_point_list) {
  // Add initial pose
  pose_list_.push_back(Pose{});

  AddPointToCells(0, Pose::Identity(), timed_point_list);
  for (auto& [key, cell] : grid_map_.cell_list) UpdateCell(&cell);

  previous_pose_.setIdentity();

  return true;
}

void GridOdometer::AddPointToCells(
    const uint64_t pose_index, const Pose& pose,
    const std::vector<TimedPoint>& timed_point_list) {
  for (const auto& timed_point : timed_point_list) {
    const Vec2 world_point = pose * timed_point.point;
    const auto& key = ComputeGridKey(world_point);

    if (grid_map_.cell_list.count(key) == 0)
      grid_map_.cell_list.insert({key, {}});
    auto& cell = grid_map_.cell_list.at(key);

    auto& point_list_per_pose = cell.point_list_per_pose;
    if (point_list_per_pose.count(pose_index) == 0)
      point_list_per_pose.insert({pose_index, {}});

    auto& point_list = point_list_per_pose.at(pose_index);
    point_list.push_back(timed_point.point);
  }
}

void GridOdometer::UpdateCell(GridCell* cell) {
  // Mean
  Vec2 sum{Vec2::Zero()};
  Mat2x2 moment{Mat2x2::Zero()};
  int num_points = 0;
  for (const auto& [key, point_list] : cell->point_list_per_pose) {
    const auto& pose = pose_list_.at(key);
    for (const auto& point : point_list) {
      const auto& world_point = pose * point;
      sum += world_point;
      moment += world_point * world_point.transpose();
      ++num_points;
    }
  }

  if (num_points < parameters_.grid_cell.min_num_point) return;

  const double inverse_num_points = 1.0 / num_points;
  cell->mean = sum * inverse_num_points;
  Mat2x2 covariance =
      moment * inverse_num_points - cell->mean * cell->mean.transpose();

  Eigen::SelfAdjointEigenSolver<Mat2x2> eigen_solver(covariance);
  const auto eigvals = eigen_solver.eigenvalues();
  const auto eigvecs = eigen_solver.eigenvectors();
  std::cerr << "eigvals: " << eigvals.transpose() << std::endl;

  // Check planarity
  const double kPlanarityThreshold{1e-3};
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

}  // namespace grid_odometer