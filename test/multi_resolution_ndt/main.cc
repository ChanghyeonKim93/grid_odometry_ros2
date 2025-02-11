#include <iostream>
#include <queue>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "ndt_updater.h"
#include "types.h"

const double focal_length = 320.0;
const double cx = 1280.0 / 2.0;
const double cy = 720.0 / 2.0;

std::vector<Vec2> GeneratePointsFromImage();

int main(int argc, char** argv) {
  //
  ndt_updater::NdtUpdater ndt_updater;

  const auto points = GeneratePointsFromImage();
  ndt_updater.AddPoints(points);
  ndt_updater.UpdateNdtMap();

  // Draw result
  cv::Mat image1 =
      cv::Mat(cv::Size(1280, 720), CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat image2 =
      cv::Mat(cv::Size(1280, 720), CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat image3 =
      cv::Mat(cv::Size(1280, 720), CV_8UC3, cv::Scalar(255, 255, 255));
  const auto ndt_map = ndt_updater.GetNdtMap();
  for (const auto& [key, ndt_root] : ndt_map) {
    const double inv_count = 1.0 / ndt_root.count;
    const Vec2 mean = ndt_root.sum * inv_count;
    const Mat2x2 cov = ndt_root.moment * inv_count - mean * mean.transpose();

    Eigen::SelfAdjointEigenSolver<Mat2x2> solver(cov);
    Vec2 evals = solver.eigenvalues();
    evals = evals.cwiseSqrt();
    if (std::isnan(evals.sum())) continue;
    const Mat2x2 evecs = solver.eigenvectors();
    const double angle = std::atan2(evecs(1, 1), evecs(0, 1)) * 180.0 / M_PI;
    Vec2 center;
    center.x() = mean.x() * focal_length + cx;
    center.y() = mean.y() * focal_length + cy;
    cv::ellipse(image1, cv::Point2f(center.x(), center.y()),
                cv::Size(focal_length * evals(1), focal_length * evals(0)),
                angle, 0.0, 360.0, cv::Scalar(255, 0, 0), 2);
    cv::drawMarker(image1, cv::Point2f(center.x(), center.y()),
                   cv::Scalar(0, 0, 0), cv::MARKER_CROSS, 15, 1);
  }
  for (const auto& [key, ndt_root] : ndt_map) {
    for (int i = 0; i < 4; ++i) {
      const auto& node = ndt_root.child_nodes[i];
      if (node == nullptr) continue;

      const double inv_count = 1.0 / node->count;
      const Vec2 mean = node->sum * inv_count;
      const Mat2x2 cov = node->moment * inv_count - mean * mean.transpose();

      Eigen::SelfAdjointEigenSolver<Mat2x2> solver(cov);
      Vec2 evals = solver.eigenvalues();
      evals = evals.cwiseSqrt();
      if (std::isnan(evals.sum())) continue;
      const Mat2x2 evecs = solver.eigenvectors();
      const double angle = std::atan2(evecs(1, 1), evecs(0, 1)) * 180.0 / M_PI;
      Vec2 center;
      center.x() = mean.x() * focal_length + cx;
      center.y() = mean.y() * focal_length + cy;
      cv::ellipse(image2, cv::Point2f(center.x(), center.y()),
                  cv::Size(focal_length * evals(1), focal_length * evals(0)),
                  angle, 0.0, 360.0, cv::Scalar(255, 0, 0), 1);
      cv::drawMarker(image2, cv::Point2f(center.x(), center.y()),
                     cv::Scalar(0, 0, 0), cv::MARKER_CROSS, 7, 1);
    }
  }

  for (const auto& [key, ndt_root] : ndt_map) {
    const ndt_updater::NdtNode* node = &ndt_root;
    std::deque<const ndt_updater::NdtNode*> q;
    q.push_back(node);
    while (!q.empty()) {
      auto node = q.front();
      q.pop_front();

      if (node == nullptr) continue;
      const double inv_count = 1.0 / node->count;
      const Vec2 mean = node->sum * inv_count;
      const Mat2x2 cov = node->moment * inv_count - mean * mean.transpose();

      Eigen::SelfAdjointEigenSolver<Mat2x2> solver(cov);
      Vec2 evals = solver.eigenvalues();
      evals = evals.cwiseSqrt();
      if (std::isnan(evals.sum())) continue;
      evals(0) = std::max(evals(0), 0.001);

      if (node->depth == 5) {
        const Mat2x2 evecs = solver.eigenvectors();
        const double angle =
            std::atan2(evecs(1, 1), evecs(0, 1)) * 180.0 / M_PI;
        Vec2 center;
        center.x() = mean.x() * focal_length + cx;
        center.y() = mean.y() * focal_length + cy;
        cv::ellipse(image3, cv::Point2f(center.x(), center.y()),
                    cv::Size(focal_length * evals(1) * 1.5,
                             focal_length * evals(0) * 1.5),
                    angle, 0.0, 360.0, cv::Scalar(255, 0, 0), 1);
        cv::drawMarker(image3, cv::Point2f(center.x(), center.y()),
                       cv::Scalar(0, 0, 0), cv::MARKER_CROSS, 7, 1);
      } else {
        if (evals(1) / evals(0) > 10.0) {
          const Mat2x2 evecs = solver.eigenvectors();
          const double angle =
              std::atan2(evecs(1, 1), evecs(0, 1)) * 180.0 / M_PI;
          Vec2 center;
          center.x() = mean.x() * focal_length + cx;
          center.y() = mean.y() * focal_length + cy;
          cv::ellipse(image3, cv::Point2f(center.x(), center.y()),
                      cv::Size(focal_length * evals(1) * 1.5,
                               focal_length * evals(0) * 1.5),
                      angle, 0.0, 360.0, cv::Scalar(255, 0, 0), 1);
          cv::drawMarker(image3, cv::Point2f(center.x(), center.y()),
                         cv::Scalar(0, 0, 0), cv::MARKER_CROSS, 7, 1);
        } else {
          for (int i = 0; i < 4; ++i) {
            if (node->child_nodes[i] == nullptr) continue;
            q.push_back(node->child_nodes[i]);
          }
        }
      }
    }

    for (int i = 0; i < 4; ++i) {
      const auto& node = ndt_root.child_nodes[i];
      if (node == nullptr) continue;
    }
  }

  cv::namedWindow("result1");
  cv::imshow("result1", image1);
  cv::namedWindow("result2");
  cv::imshow("result2", image2);
  cv::namedWindow("result3");
  cv::imshow("result3", image3);
  cv::waitKey(0);

  return 0;
}

std::vector<Vec2> GeneratePointsFromImage() {
  const double inv_f = 1.0 / focal_length;
  cv::Mat image = cv::imread("../map.png", cv::IMREAD_GRAYSCALE);
  cv::namedWindow("map");
  cv::imshow("map", image);
  cv::waitKey(0);

  std::vector<Vec2> points;
  for (int v = 0; v < image.rows; ++v) {
    for (int u = 0; u < image.cols; ++u) {
      if (image.at<uint8_t>(v, u) < 255) {
        Vec2 point;
        point.x() = (u - cx) * inv_f;
        point.y() = (v - cy) * inv_f;
        points.push_back(point);
      }
    }
  }

  std::cerr << "# points: " << points.size() << std::endl;
  return points;
}