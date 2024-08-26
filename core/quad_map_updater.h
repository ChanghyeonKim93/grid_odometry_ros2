#ifndef GRID_ODOMETER_CORE_QUAD_MAP_UPDATER_H_
#define GRID_ODOMETER_CORE_QUAD_MAP_UPDATER_H_

#include "core/types.h"

namespace grid_odometer {
namespace quad_map_updater {

struct Parameters {
  double grid_size{0.3};
  int max_depth{2};  // 0.3, 0.15, 0.075
  double large_eigenvalue_threshold{0.05};
  double min_eigenvalue_ratio{0.01};
  struct {
    int min_num_point_to_check_linearity{10};
    int max_num_point_in_quad_base{50};
  } quad;
};

class QuadMapUpdater {
 public:
  explicit QuadMapUpdater(const Parameters& parameters);

  void UpdateQuadMap(const std::vector<Point>& local_point_list,
                     QuadMap* quad_map);

  std::vector<Point> GenerateRayPointList(const Point& p0, const Point& p1);

 private:
  uint64_t ComputeGridKey(const Point& point);
  uint8_t FindQuadrant(const Point& point, const Point& center);

  const Parameters parameters_;
};

}  // namespace quad_map_updater
}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_QUAD_MAP_UPDATER_H_