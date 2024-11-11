#include <iostream>
#include <stack>
#include <unordered_map>

#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"

#include "types.h"

using namespace grid_mapper;

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

auto compute_mean_and_covariance =
    [](const std::vector<Vec2>& all_point_list,
       const std::vector<int>& point_index_list) {
      MeanAndCovariance res;
      double xx = 0.0, xy = 0.0, yy = 0.0;
      for (const auto& point_index : point_index_list) {
        const auto& point = all_point_list.at(point_index);
        res.mean += point;
        res.xx += point.x() * point.x();
        res.xy += point.x() * point.y();
        res.yy += point.y() * point.y();
      }
      const size_t num_points = point_index_list.size();
      const double inverse_num_points = 1.0 / num_points;
      res.mean = res.mean * inverse_num_points;
      res.xx = res.xx * inverse_num_points - res.mean.x() * res.mean.x();
      res.xy = res.xy * inverse_num_points - res.mean.x() * res.mean.y();
      res.yy = res.yy * inverse_num_points - res.mean.y() * res.mean.y();

      return res;
    };

auto eigendecompose = [](const double a, const double c, const double b) {
  const double del = std::sqrt(-4 * c * c + (a - b) * (a - b));

  EigenDecomposeResult res;
  res.l1 = (a + b - del) * 0.5;
  res.l2 = (a + b + del) * 0.5;
  if (std::abs(c) < 1e-7) {
    res.v1 << 1.0, 0.0;
    res.v2 << 0.0, 1.0;
  } else {
    res.v1.x() = (res.l2 - b) / c;
    res.v1.y() = 1.0;
    res.v1.normalize();
    res.v2.x() = (res.l1 - b) / c;
    res.v2.y() = 1.0;
    res.v2.normalize();
  }

  return res;
};

double grid_resolution{0.4};
int max_depth{3};
std::vector<int> max_num_points{40, 20, 10, 5};

std::unordered_map<int64_t, GridCell> grid_cell_map_;

int ComputeGridCellKey(const Vec2& point, const double grid_resolution) {
  const double inverse_resolution = 1.0 / grid_resolution;
  int x_key = std::floor(point.x() * inverse_resolution);
  int y_key = std::floor(point.y() * inverse_resolution);
  x_key = (x_key < 0) ? -2 * x_key : 2 * x_key + 1;
  y_key = (y_key < 0) ? -2 * y_key : 2 * y_key + 1;
  const int key = (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key + 1;
  return key;
}

int FindQuadrant(const Vec2& point, const QuadNodePtr& node) {
  const double half_size = node->size * 0.5;
  const double center_x = node->x + half_size;
  const double center_y = node->y + half_size;
  const int is_right = point.x() > center_x;
  const int is_bottom = point.y() > center_y;
  const int quadrant_index = is_right + is_bottom * 2;
  return quadrant_index;
}

void AddPoint(const Vec2& point,
              std::unordered_map<int64_t, GridCell>* grid_cell_map) {
  const int64_t key = ComputeGridCellKey(point, grid_resolution);
  if (!grid_cell_map->count(key)) {
    const double inv_res = 1.0 / grid_resolution;
    const double x = std::floor(point.x() * inv_res) * grid_resolution;
    const double y = std::floor(point.y() * inv_res) * grid_resolution;
    GridCell grid_cell;
    grid_cell.root = std::make_shared<QuadNode>(x, y, grid_resolution, 0);
    grid_cell_map->insert({key, grid_cell});
  }
  auto& grid_cell = grid_cell_map->at(key);
  grid_cell.point_list.push_back(point);
  const int point_index = static_cast<int>(grid_cell.point_list.size() - 1);

  QuadNodePtr node = grid_cell.root;
  while (true) {
    node->point_index_list.push_back(point_index);
    if (node->point_index_list.size() < 5) break;
    if (node->depth == max_depth) break;

    if (node->point_index_list.size() > max_num_points[node->depth]) {
      // Check linearity
      const auto mean_and_cov = compute_mean_and_covariance(
          grid_cell.point_list, node->point_index_list);
      const auto eig_res =
          eigendecompose(mean_and_cov.xx, mean_and_cov.xy, mean_and_cov.yy);
      node->ndt_center = mean_and_cov.mean;
      node->ndt_axis << std::abs(eig_res.l1), std::abs(eig_res.l2);
      node->ndt_angle = std::atan2(eig_res.v1.y(), eig_res.v1.x());
      if (std::abs(eig_res.l2 / eig_res.l1) > 20.0) break;

      // traverse down
      const int child_idx = FindQuadrant(point, node);
      auto& child_node = node->child_node_list[child_idx];
      const double half_size = node->size * 0.5;
      const double x_list[4] = {node->x, node->x + half_size, node->x,
                                node->x + half_size};
      const double y_list[4] = {node->y, node->y, node->y + half_size,
                                node->y + half_size};
      if (!child_node)
        child_node = std::make_shared<QuadNode>(
            x_list[child_idx], y_list[child_idx], half_size, node->depth + 1);
      node = child_node;
    } else {
      break;
    }
  }
}

int main(int argc, char** argv) {
  const double step = 0.01;
  const double x_shift = 0.1324;
  const double y_shift = 0.121;
  std::vector<Vec2> point_list;
  for (int i = 0; i < 40; ++i) point_list.emplace_back(step * i, 0.0);
  for (int i = 0; i < 9; ++i) point_list.emplace_back(step * 40, step * i);
  for (int i = 0; i < 30; ++i)
    point_list.emplace_back(step * (40 + i), step * 9);
  for (int i = 0; i < 50; ++i)
    point_list.emplace_back(step * (70), step * (9 + i));
  for (int i = 0; i < 12; ++i)
    point_list.emplace_back(step * (70 + i), step * (59));
  for (int i = 0; i < 5; ++i)
    point_list.emplace_back(step * (82), step * (59 + i));
  for (int i = 0; i < 90; ++i)
    point_list.emplace_back(step * (82 - i), step * (64));

  for (auto& point : point_list) point += Vec2(x_shift, y_shift);

  for (const auto& point : point_list) AddPoint(point, &grid_cell_map_);
  cv::Mat image = cv::Mat::zeros(640, 640, CV_8UC3);

  constexpr double scaler = 500.0;
  cv::Scalar point_color(255, 0, 0);
  cv::Scalar line_color(122, 122, 122);
  cv::Scalar ellipse_color(0, 0, 255);
  for (const auto& point : point_list) {
    cv::circle(image, cv::Point2f(point.x() * scaler, point.y() * scaler), 2,
               point_color);
  }
  for (const auto& [key, grid_cell] : grid_cell_map_) {
    auto node = grid_cell.root;
    std::stack<QuadNodePtr> st;
    st.push(node);
    while (!st.empty()) {
      auto node = st.top();
      st.pop();
      cv::rectangle(image,
                    cv::Rect2f(cv::Point2f(node->x * scaler, node->y * scaler),
                               cv::Point2f((node->x + node->size) * scaler,
                                           (node->y + node->size) * scaler)),
                    line_color, 1);
      cv::circle(image,
                 cv::Point2f(node->ndt_center.x() * scaler,
                             node->ndt_center.y() * scaler),
                 4, ellipse_color, 1);
      cv::ellipse(image,
                  cv::Point2f(node->ndt_center.x() * scaler,
                              node->ndt_center.y() * scaler),
                  cv::Size2f(node->ndt_axis.x() * 6.0 * scaler,
                             node->ndt_axis.y() * 6.0 * scaler),
                  node->ndt_angle / M_PI * 180, 0, 360, ellipse_color, 2);
      for (int i = 0; i < 4; ++i) {
        auto child_node = node->child_node_list[i];
        if (!child_node) continue;
        st.push(child_node);
      }
    }
  }
  cv::namedWindow("image");
  cv::imshow("image", image);
  cv::waitKey(0);
  return 0;
}