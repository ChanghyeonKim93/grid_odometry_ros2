
#include "core/quad_map_updater.h"
#include "core/types.h"

#include <stack>

namespace grid_odometer {
namespace quad_map_updater {

explicit QuadMapUpdater::QuadMapUpdater(const Parameters& parameters)
    : parameters_{parameters} {}

void QuadMapUpdater::UpdateQuadMap(const std::vector<Point>& local_point_list,
                                   QuadMap* quad_map) {
  constexpr size_t kMinNumPointToCheckLinearity{10};
  constexpr size_t kMaxNumPointInQuadBase{50};
  const int max_depth = parameters_.max_depth;
  const double grid_size = parameters_.grid_size;
  const double inverse_grid_size = 1.0 / grid_size;

  /*
    [Algorithm]
    1) Warp point and insert point to the quad map
    2) For all quad base, pop front point if # points > kMaxNumPointInQuadBase.
    3) Make stack for leaf nodes
    4) iterate stack until stack empty
      If # points > kMinNumPointsInQuad, check linearity.
        If it is linear, make line.
        Else
          If depth < kMaxDepth, make the child and push point to the child, and
           stack the child ptr
      Else continue; // just stack points
  */

  // 1) Insert points
  for (const auto& point : local_point_list) {
    const uint64_t key = ComputeGridKey(point);
    if (!quad_map->quad_base_list.count(key)) {
      quad_map->quad_base_list.insert({key, {}});
      auto& quad_base = quad_map->quad_base_list.at(key);
      quad_base.root = new Quad(&quad_base.point_list);
      auto xi = static_cast<int64_t>(std::round(point.x() * inverse_grid_size));
      auto yi = static_cast<int64_t>(std::round(point.y() * inverse_grid_size));
      quad_base.root->left_bottom.x() = grid_size * xi;
      quad_base.root->left_bottom.y() = grid_size * yi;
      quad_base.root->right_top.x() =
          quad_base.root->left_bottom.x() + grid_size;
      quad_base.root->right_top.y() =
          quad_base.root->left_bottom.y() + grid_size;
    }

    auto& quad_base = quad_map->quad_base_list.at(key);
    quad_base.point_list.push_back(point);
    const size_t point_index = quad_base.point_list.size() - 1;

    // Find quad of the point
    Quad* quad_ptr = quad_base.root;
    while (true) {
      const Point center_point =
          (quad_ptr->left_bottom + quad_ptr->right_top) * 0.5;
      const uint8_t quadrant_index = FindQuadrant(point, center_point);
      if (!quad_ptr->child_list[quadrant_index]) break;
      quad_ptr = quad_ptr->child_list[quadrant_index];
    }
    quad_ptr->point_index_list.push_back(point_index);
  }

  // 2) For all quad base, pop front point if # points > kMaxNumPointInQuadBase.
  for (auto& [key, quad_base] : quad_map->quad_base_list) {
    while (quad_base.point_list.size() >= kMaxNumPointInQuadBase)
      quad_base.point_list.pop_front();
  }

  // 3) Make stack for leaf nodes
  std::stack<Quad*> quad_ptr_stack;
  for (auto& [key, quad_base] : quad_map->quad_base_list) {
    std::stack<Quad*> quad_ptr_stack_for_traverse;
    quad_ptr_stack_for_traverse.push(quad_base.root);
    while (!quad_ptr_stack_for_traverse.empty()) {
      Quad* qp = quad_ptr_stack_for_traverse.top();
      quad_ptr_stack_for_traverse.pop();

      if (!qp->point_index_list.empty()) {
        quad_ptr_stack.push(qp);
        continue;
      }
      if (qp->child_list[0])
        quad_ptr_stack_for_traverse.push(qp->child_list[0]);
      if (qp->child_list[1])
        quad_ptr_stack_for_traverse.push(qp->child_list[1]);
      if (qp->child_list[2])
        quad_ptr_stack_for_traverse.push(qp->child_list[2]);
      if (qp->child_list[3])
        quad_ptr_stack_for_traverse.push(qp->child_list[3]);
    }
  }
  /*
  4) iterate stack until stack empty
      If # points > kMinNumPointsInQuad, check linearity.
        If it is linear, make line.
        Else
          If depth < kMaxDepth, make the child and push point to the child,
          and stack the child ptr
      Else continue; // just stack points
  */

  struct MeanAndCovariance {
    Vec2 mean{Vec2::Zero()};
    double xx{0.0};
    double xy{0.0};
    double yy{0.0};
  };
  struct EigenDecomposeResult {
    double l1{0.0};
    double l2{0.0};
    Vec2 v1{Vec2::Zero()};
    Vec2 v2{Vec2::Zero()};
  };

  auto compute_mean_and_covariance = [](const std::vector<Vec2>& point_list) {
    MeanAndCovariance res;
    double xx = 0.0, xy = 0.0, yy = 0.0;
    for (const auto& point : point_list) {
      res.mean += point;
      res.xx += point.x() * point.x();
      res.xy += point.x() * point.y();
      res.yy += point.y() * point.y();
    }
    const size_t num_points = point_list.size();
    const double inverse_num_points = 1.0 / num_points;
    res.mean = res.mean * inverse_num_points;
    res.xx = res.xx * inverse_num_points - res.mean.x() * res.mean.x();
    res.xy = res.xy * inverse_num_points - res.mean.x() * res.mean.y();
    res.yy = res.yy * inverse_num_points - res.mean.y() * res.mean.y();

    return res;
  };

  auto eigendecompose = [](const double xx, const double xy, const double yy) {
    const double b = -xx - yy;
    const double b_sq = b * b;
    const double c = xx * yy - xy * xy;
    const double sqrt_b_sq_m4c = std::sqrt(b_sq - 4.0 * c);

    EigenDecomposeResult res;
    res.l1 = 0.5 * (-b + sqrt_b_sq_m4c);
    res.l2 = 0.5 * (-b - sqrt_b_sq_m4c);
    res.v1.x() = xy;
    res.v1.y() = (res.l1 - xx);
    res.v1.normalize();
    res.v2.x() = xy;
    res.v2.y() = (res.l2 - xx);
    res.v2.normalize();
    if (std::abs(res.l1) < std::abs(res.l2)) {
      const double tmp = res.l1;
      res.l1 = res.l2;
      res.l2 = tmp;
      const Vec2 tmp_v = res.v1;
      res.v1 = res.v2;
      res.v2 = tmp_v;
    }
    return res;
  };

  while (!quad_ptr_stack.empty()) {
    Quad* qp = quad_ptr_stack.top();
    quad_ptr_stack.pop();
    if (qp->point_index_list.size() > kMinNumPointToCheckLinearity) {
      // Check linearity
      Point mean{Point::Zero()};
      double xx = 0.0, xy = 0.0, yy = 0.0;
      for (const auto& point_index : qp->point_index_list) {
        const auto& point = qp->point_list_ptr->at(point_index);
        mean += point;
        xx += point.x() * point.x();
        xy += point.x() * point.y();
        yy += point.y() * point.y();
      }
      const size_t num_points = qp->point_index_list.size();
      const double inverse_num_points = 1.0 / num_points;

      mean *= inverse_num_points;
      const double cov_xx = xx * inverse_num_points - mean.x() * mean.x();
      const double cov_xy = xy * inverse_num_points - mean.x() * mean.y();
      const double cov_yy = yy * inverse_num_points - mean.y() * mean.y();
      const auto eigen_res = eigendecompose(cov_xx, cov_xy, cov_yy);
      const bool is_linear =
          std::abs(eigen_res.l1) > parameters_.large_eigenvalue_threshold &&
          std::abs(eigen_res.l2) <
              std::abs(eigen_res.l1) * parameters_.min_eigenvalue_ratio;
      if (is_linear) {
        // Find major eigen vector to make it direction.
        qp->direction = eigen_res.v1;
        qp->mean = mean;
      } else {
        // Divide
        for (const auto& point_index : qp->point_index_list) {
          const auto& point = qp->point_list_ptr->at(point_index);
          const Vec2 center = (qp->left_bottom + qp->right_top) * 0.5;
          const auto quadrant_index = FindQuadrant(point, center);
          if (!qp->child_list[quadrant_index]) {
            qp->child_list[quadrant_index] = new Quad(qp->point_list_ptr);
          }
        }
        qp->point_index_list.clear();
      }
    }
  }
}

uint64_t QuadMapUpdater::ComputeGridKey(const Point& point) {
  static double inverse_grid_size = 1.0 / parameters_.grid_size;
  auto xi = static_cast<int64_t>(std::round(point.x() * inverse_grid_size));
  auto yi = static_cast<int64_t>(std::round(point.y() * inverse_grid_size));
  const uint64_t x_key = (xi > 0) ? 2 * xi : -2 * xi + 1;
  const uint64_t y_key = (yi > 0) ? 2 * yi : -2 * yi + 1;
  const uint64_t key = ((x_key + y_key) * (x_key + y_key + 1)) >> 1 + y_key;
  return key;
}

uint8_t QuadMapUpdater::FindQuadrant(const Point& point, const Point& center) {
  /*
   y
   ^
   | 2  3
   | 0  1
    ------> x
  */
  const uint8_t x_index = point.x() > center.x();
  const uint8_t y_index = point.y() > center.y();
  const uint8_t quadrant_number = static_cast<uint8_t>(y_index << 1 | x_index);
  return quadrant_number;
}

}  // namespace quad_map_updater
}  // namespace grid_odometer
