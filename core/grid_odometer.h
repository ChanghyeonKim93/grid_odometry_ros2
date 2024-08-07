#ifndef GRID_ODOMETER_CORE_GRID_ODOMETER_H_
#define GRID_ODOMETER_CORE_GRID_ODOMETER_H_

#include "eigen3/Eigen/Dense"

#include "core/bridge.h"
#include "core/types.h"

namespace grid_odometer {

struct Parameters {
  double grid_resolution{1.0};  // [m]
  struct {
    int max_depth{3};
    int min_num_point{10};
  } grid_cell;
};

class GridOdometer {
 public:
  explicit GridOdometer(const Parameters& parameters);

  void Update(const bridge::LaserScan& bridge_laser_scan);

  bridge::GridMap GetGridMap() const;

 private:
  uint64_t ComputeGridKey(const Point point);
  bool InitializeGridMap(
      const std::vector<grid_odometer::TimedPoint>& timed_point_list);
  void AddPointToCells(const uint64_t pose_index, const Pose& pose,
                       const std::vector<TimedPoint>& timed_point_list);
  void UpdateCell(GridCell* cell);

  Pose EstimatedPose(const Pose& initial_pose,
                     const std::vector<TimedPoint>& timed_point_list);

  std::vector<TimedPoint> ConvertToTimedPointList(
      const std::vector<bridge::TimedPoint>& bridge_timed_point_list);

  const Parameters parameters_;

  struct {
    bool is_initialized{false};
  } state_;

  std::vector<Pose> pose_list_;
  Pose previous_pose_{Pose::Identity()};

  GridMap grid_map_;
};

}  // namespace grid_odometer

#endif  // GRID_ODOMETER_CORE_GRID_ODOMETER_H_