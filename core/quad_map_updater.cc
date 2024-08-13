
#include "core/quad_map_updater.h"
#include "core/types.h"

namespace grid_odometer {
namespace quad_map_updater {

explicit QuadMapUpdater::QuadMapUpdater(const Parameters& parameters)
    : parameters_{parameters} {}

void QuadMapUpdater::UpdateQuadMap(const int pose_index, const Pose& local_pose,
                                   const std::vector<Point>& point_list,
                                   QuadMap* quad_map) {
  auto& cell_list = grid_map->cell_list;
  for (const auto& local_point : local_point_list) {
    const Point point_in_reference = pose_in_reference * local_point;
    const uint64_t key = ComputeGridKey(point_in_reference);
    if (!cell_list.count(key)) cell_list.insert({key, {}});

    auto& point_list_per_pose = cell_list.at(key).point_list_for_pose_index;
    if (!point_list_per_pose.count(pose_index))
      point_list_per_pose.insert({pose_index, {}});
    point_list_per_pose.at(pose_index).push_back(local_point);
  }
  // DFS. 그리드 셀 깊은 레벨에 셀이 있다면, 포함된 점의 개수를 센다.
  // 만약 threshold보다 크다면, planarity 계산한다.
  // planarity가 만족되지 않고,
}

uint64_t QuadMapUpdater::ComputeGridKey(const Point& point) {
  static double inverse_grid_size = 1.0 / parameters_.grid_size;
  auto xi = static_cast<int64_t>(std::round(point.x() * inverse_grid_size));
  auto yi = static_cast<int64_t>(std::round(point.y() * inverse_grid_size));
  const uint64_t x_key = (xi > 0) ? 2 * xi : -2 * xi + 1;
  const uint64_t y_key = (yi > 0) ? 2 * yi : -2 * yi + 1;
  const uint64_t key = (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key;
  return key;
}

}  // namespace quad_map_updater
}  // namespace grid_odometer
