#ifndef GRID_ODOMETER_CORE_GRID_MAP_UPDATER_H_
#define GRID_ODOMETER_CORE_GRID_MAP_UPDATER_H_

#include "core/types.h"

namespace grid_odometer {
namespace grid_map_updater {

struct Parameters {
  double resolution{0.05};
};

class GridMapUpdater {
 public:
  explicit GridMapUpdater(const Parameters& parameters);

  void UpdateGridMap(const Pose& pose_in_grid_map,
                     const std::vector<Point>& local_point_list,
                     GridMap* grid_map);
  void ExtendGridMap(const Pose& pose_in_grid_map,
                     const std::vector<Point>& local_point_list,
                     GridMap* grid_map);

  std::vector<Point> GenerateRayPointList(const Point& p0, const Point& p1);

 private:
  int ComputeGridCellIndex(const Point& point_in_grid_map,
                           const GridMap& grid_map);

  const Parameters parameters_;
};

}  // namespace grid_map_updater
}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_GRID_MAP_UPDATER_H_