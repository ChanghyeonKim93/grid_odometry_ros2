#include <iostream>
#include <vector>

#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;
using Mat2x2 = Eigen::Matrix2d;

struct EigenDecomposeResult {
  double l1{0.0};
  double l2{0.0};
  Vec2 v1{Vec2::Zero()};
  Vec2 v2{Vec2::Zero()};
};

struct MeanAndCovariance {
  Vec2 mean{Vec2::Zero()};
  double xx{0.0};
  double xy{0.0};
  double yy{0.0};
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

int main(int argc, char** argv) {
  std::vector<Vec2> point_list;
  point_list.emplace_back(0.0, 0.0);
  point_list.emplace_back(0.1, 0.1);
  point_list.emplace_back(0.2, 0.2);
  point_list.emplace_back(0.3, 0.3);
  point_list.emplace_back(0.4, 0.4);

  auto mean_cov_res = compute_mean_and_covariance(point_list);
  auto eigdecompose_res =
      eigendecompose(mean_cov_res.xx, mean_cov_res.xy, mean_cov_res.yy);

  std::cerr << mean_cov_res.mean.transpose() << std::endl;
  std::cerr << eigdecompose_res.l1 << std::endl;
  std::cerr << eigdecompose_res.l2 << std::endl;
  std::cerr << eigdecompose_res.v1 << std::endl;
  std::cerr << eigdecompose_res.v2 << std::endl;

  return 0;
}