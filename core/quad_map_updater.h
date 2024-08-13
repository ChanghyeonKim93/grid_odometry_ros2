#ifndef GRID_ODOMETER_CORE_QUAD_MAP_UPDATER_H_
#define GRID_ODOMETER_CORE_QUAD_MAP_UPDATER_H_

#include "core/types.h"

namespace grid_odometer {
namespace quad_map_updater {

struct Parameters {
  double grid_size{0.3};
  int max_depth{3};
  double min_eigenvalue_ratio{0.01};
};

class QuadMapUpdater {
 public:
  explicit QuadMapUpdater(const Parameters& parameters);

  void UpdateQuadMap(const int pose_index, const Pose& local_pose,
                     const std::vector<Point>& point_list, QuadMap* quad_map);

  std::vector<Point> GenerateRayPointList(const Point& p0, const Point& p1);

 private:
  uint64_t ComputeGridKey(const Point& point);

  const Parameters parameters_;
};

}  // namespace quad_map_updater
}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_QUAD_MAP_UPDATER_H_