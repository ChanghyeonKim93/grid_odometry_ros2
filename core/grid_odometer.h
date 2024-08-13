#ifndef GRID_ODOMETER_CORE_GRID_ODOMETER_H_
#define GRID_ODOMETER_CORE_GRID_ODOMETER_H_

#include <memory>

#include "eigen3/Eigen/Dense"

#include "core/bridge.h"
#include "core/grid_map_updater.h"
#include "core/quad_map_updater.h"
#include "core/types.h"

namespace grid_odometer {

struct Parameters {
  grid_map_updater::Parameters grid_map_updater;
  quad_map_updater::Parameters quad_map_updater;
};

class GridOdometer {
 public:
  explicit GridOdometer(const Parameters& parameters);

  void Update(const bridge::TimedPointCloud& bridge_laser_scan);

  bridge::GridMap GetGridMap() const;

  std::vector<bridge::Point> GetRayPointList(
      const bridge::TimedPointCloud& bridge_laser_scan);

 private:
  bool InitializeGridMap(
      const std::vector<grid_odometer::TimedPoint>& timed_point_list);

  Pose EstimatedPose(const Pose& initial_pose,
                     const std::vector<TimedPoint>& timed_point_list);

  std::vector<TimedPoint> ConvertToTimedPointList(
      const std::vector<bridge::TimedPoint>& bridge_timed_point_list);
  std::vector<Point> ConvertToPointList(
      const std::vector<bridge::TimedPoint>& bridge_timed_point_list);

  const Parameters parameters_;

  struct {
    bool is_initialized{false};
  } state_;

  std::vector<Pose> pose_list_;
  Pose previous_pose_{Pose::Identity()};

  GridMap current_grid_map_;

  std::unique_ptr<grid_map_updater::GridMapUpdater> grid_map_updater_;
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_GRID_ODOMETER_H_