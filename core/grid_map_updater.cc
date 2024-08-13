#include "core/grid_map_updater.h"

#include <iostream>

namespace grid_odometer {
namespace grid_map_updater {

GridMapUpdater::GridMapUpdater(const Parameters& parameters)
    : parameters_{parameters} {}

void GridMapUpdater::UpdateGridMap(
    const Pose& pose_in_reference,
    const std::vector<Point>& local_point_list, /* local points */
    GridMap* grid_map) {
  grid_map->resolution = parameters_.resolution;

  ExtendGridMap(pose_in_reference, local_point_list, grid_map);

  const double inverse_resolution = 1.0 / parameters_.resolution;
  auto& cell_list = grid_map->cell_list;
  for (const auto& local_point : local_point_list) {
    const Point hit_point = pose_in_reference * local_point;
    const auto hit_cell_index = ComputeGridCellIndex(hit_point, *grid_map);
    // Update hit
    cell_list.at(hit_cell_index).hit_count += 10;
    if (cell_list.at(hit_cell_index).hit_count > 100)
      cell_list.at(hit_cell_index).hit_count = 100;

    // Update miss
    const auto miss_point_list = GenerateRayPointList(Point::Zero(), hit_point);
    for (const auto& miss_point : miss_point_list) {
      const auto miss_cell_index = ComputeGridCellIndex(miss_point, *grid_map);

      cell_list.at(miss_cell_index).hit_count -= 20;
      if (cell_list.at(miss_cell_index).hit_count < 0)
        cell_list.at(miss_cell_index).hit_count = 0;
    }
  }
}

void GridMapUpdater::ExtendGridMap(const Pose& pose_in_grid_map,
                                   const std::vector<Point>& local_point_list,
                                   GridMap* grid_map) {
  const double inverse_resolution = 1.0 / grid_map->resolution;

  int min_u = 0;
  int min_v = 0;
  int max_u = grid_map->width - 1;
  int max_v = grid_map->height - 1;
  for (const auto& local_point : local_point_list) {
    const auto point_in_grid_map = pose_in_grid_map * local_point;
    const int u = std::floor(point_in_grid_map.x() * inverse_resolution) -
                  grid_map->origin.x();
    const int v = std::floor(point_in_grid_map.y() * inverse_resolution) -
                  grid_map->origin.y();
    if (u < min_u) min_u = u;
    if (u > max_u) max_u = u;
    if (v < min_v) min_v = v;
    if (v > max_v) max_v = v;
  }

  // copy existing cells
  Eigen::Vector2i new_origin(grid_map->origin.x() + min_u,
                             grid_map->origin.y() + min_v);
  int new_height = max_v - min_v + 1;
  int new_width = max_u - min_u + 1;

  std::vector<GridCell> new_cell_list{new_height * new_width, {-1}};
  for (size_t index = 0; index < grid_map->cell_list.size(); ++index) {
    const auto& cell = grid_map->cell_list.at(index);
    const int v = std::floor(index / grid_map->width);
    const int u = index - v * grid_map->width;
    const int new_v = v - min_v;
    const int new_u = u - min_u;
    new_cell_list.at(new_v * new_width + new_u) = cell;
  }
  grid_map->cell_list = std::move(new_cell_list);
  grid_map->height = new_height;
  grid_map->width = new_width;
  grid_map->origin = new_origin;
}

std::vector<Point> GridMapUpdater::GenerateRayPointList(const Point& p0,
                                                        const Point& p1) {
  const double grid_size = parameters_.resolution * 0.9999;

  constexpr double kEpsilon = 1e-5;
  const double x0 = p0.x(), y0 = p0.y();
  const double x1 = p1.x(), y1 = p1.y();
  const double dx = x1 - x0, dy = y1 - y0;

  std::vector<Point> ray_point_list;
  ray_point_list.reserve(200);
  if (std::abs(dx) >= std::abs(dy)) {
    const double dydx = dy / dx;
    if (dx > 0.0) {
      for (double x = x0; x <= x1; x += grid_size) {
        const double y = dydx * (x - x0) + y0;
        ray_point_list.emplace_back(x, y);
      }
    } else {
      for (double x = x0; x >= x1; x -= grid_size) {
        const double y = dydx * (x - x0) + y0;
        ray_point_list.emplace_back(x, y);
      }
    }
  } else {
    const double dxdy = dx / dy;
    if (dy > 0.0) {
      for (double y = y0; y <= y1; y += grid_size) {
        const double x = dxdy * (y - y0) + x0;
        ray_point_list.emplace_back(x, y);
      }
    } else {
      for (double y = y0; y >= y1; y -= grid_size) {
        const double x = dxdy * (y - y0) + x0;
        ray_point_list.emplace_back(x, y);
      }
    }
  }

  return ray_point_list;
}

int GridMapUpdater::ComputeGridCellIndex(const Point& point_in_grid_map,
                                         const GridMap& grid_map) {
  const double inverse_resolution = 1.0 / grid_map.resolution;
  const int u = std::floor(point_in_grid_map.x() * inverse_resolution) -
                grid_map.origin.x();
  const int v = std::floor(point_in_grid_map.y() * inverse_resolution) -
                grid_map.origin.y();
  const int index = v * grid_map.width + u;
  return index;
}

}  // namespace grid_map_updater
}  // namespace grid_odometer
