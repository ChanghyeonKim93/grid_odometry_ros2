#include <iostream>
#include <random>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"

namespace {

std::random_device random_generator;

};  // namespace

const int image_height{400};
const int image_width{1600};

const double map_range[2] = {0.0, 30.0};
const double observation_range[2] = {-20.0, 20.0};
constexpr double kPoseStep{0.05};
constexpr double kPoseStart{7.1};
constexpr double kPoseEnd{12.45};
constexpr double kOdometryPositionNoiseStd{0.02};

struct Parameters {
  double resolution{0.02};
  int max_num_particles{2000};
  int min_count{3};
  int max_iterations{100};
};

Parameters parameters;

using Pose = double;
using Position = double;

struct Particle {
  Pose pose;
  double weight{0.0};
};

using GridMap = std::vector<uint8_t>;

int ComputeGridMapIndex(const Position world_position, const double resolution);
GridMap GenerateGridMap();
std::vector<std::vector<Position>> GenerateObservationList(
    const std::vector<Pose>& pose_list, const GridMap& grid_map);
void InitializeParticleList(const int num_particles,
                            std::vector<Particle>* particle_list);
double EvaluateParticle(const Particle& particle,
                        const std::vector<Position>& observation_list,
                        const GridMap& grid_map);
double InterpolateGridMap(const Position world_position,
                          const GridMap& grid_map);
void ResampleParticles(const int max_num_particles,
                       std::vector<Particle>* particle_list);
Pose Localize(const GridMap& grid_map,
              const std::vector<Position>& observation_list,
              std::vector<Particle>* particle_list);
void WarpParticlesWithNoise(const Pose& pose_delta, const double pose_noise_std,
                            std::vector<Particle>* particle_list);
void DrawParticles(const std::vector<Particle>& particle_list, const Pose& pose,
                   const GridMap& grid_map, cv::Mat* image);

int main(int argc, char** argv) {
  /*
    GENERATE SIMULATION DATA
  */
  std::normal_distribution<double> normal_dist(0.0, kOdometryPositionNoiseStd);
  std::vector<Pose> true_pose_list;
  for (double x = kPoseStart; x <= kPoseEnd; x += kPoseStep)
    true_pose_list.push_back(x);
  std::vector<Pose> odometry_pose_list;
  odometry_pose_list.push_back(true_pose_list.front());
  for (size_t index = 1; index < true_pose_list.size(); ++index) {
    const Pose delta_pose =
        true_pose_list.at(index) - true_pose_list.at(index - 1);
    odometry_pose_list.push_back(odometry_pose_list.back() + delta_pose +
                                 normal_dist(random_generator));
  }

  const auto grid_map = GenerateGridMap();
  const auto observation_lists =
      GenerateObservationList(true_pose_list, grid_map);

  /*
    Monte Carlo Localization with Motion Prior
  */
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  std::vector<Particle> particle_list;
  for (size_t pose_index = 0; pose_index < odometry_pose_list.size();
       ++pose_index) {
    const auto& observation_list = observation_lists.at(pose_index);
    if (pose_index == 0) {
      // Initialize particle list
      InitializeParticleList(parameters.max_num_particles, &particle_list);
      const auto estimated_pose =
          Localize(grid_map, observation_list, &particle_list);
      std::cerr << "True pose: " << true_pose_list.at(pose_index) << std::endl;
      std::cerr << "Estimated pose: " << estimated_pose << std::endl;
      continue;
    }

    const Pose pose_delta =
        true_pose_list.at(pose_index) - true_pose_list.at(pose_index - 1);
    WarpParticlesWithNoise(pose_delta, 0.05, &particle_list);
    const auto estimated_pose =
        Localize(grid_map, observation_list, &particle_list);
    std::cerr << "True pose: " << true_pose_list.at(pose_index) << std::endl;
    std::cerr << "Estimated pose: " << estimated_pose << std::endl;
  }

  return 0;
}

int ComputeGridMapIndex(const Position world_position,
                        const double resolution) {
  if (world_position < map_range[0] || world_position > map_range[1]) return -1;
  const int index = static_cast<int>(world_position / resolution);
  return index;
}

GridMap GenerateGridMap() {
  GridMap grid_map;
  double range[6][2] =  //
      {{0.41, 4.23},    //
       {5.10, 5.56},    //
       {7.2, 9.5},      //
       {10.41, 12.23},  //
       {15.11, 17.41},  //
       {25.11, 29.41}};

  grid_map.resize(
      std::ceil((map_range[1] - map_range[0]) / parameters.resolution), 0);
  for (int i = 0; i < 6; ++i)
    for (double x = range[i][0]; x <= range[i][1]; x += parameters.resolution)
      grid_map.at(ComputeGridMapIndex(x, parameters.resolution)) = 255;

  return grid_map;
}

std::vector<std::vector<Position>> GenerateObservationList(
    const std::vector<Pose>& pose_list, const GridMap& grid_map) {
  std::vector<std::vector<Position>> observation_lists;
  observation_lists.resize(pose_list.size(), std::vector<Position>());

  for (size_t pose_index = 0; pose_index < pose_list.size(); ++pose_index) {
    const auto& pose = pose_list.at(pose_index);
    auto& observation_list = observation_lists.at(pose_index);
    for (size_t grid_index = 0; grid_index < grid_map.size(); ++grid_index) {
      if (!grid_map.at(grid_index)) continue;

      const Position local_position = grid_index * parameters.resolution - pose;
      if (local_position < observation_range[0] ||
          local_position > observation_range[1])
        continue;
      observation_list.push_back(local_position);
    }
  }
  return observation_lists;
}

void InitializeParticleList(const int num_particles,
                            std::vector<Particle>* particle_list) {
  if (particle_list == nullptr) return;
  std::uniform_real_distribution<double> uniform_dist(map_range[0],
                                                      map_range[1]);
  const double uniform_weight = 1.0 / num_particles;
  particle_list->clear();
  particle_list->reserve(num_particles);
  for (int i = 0; i < num_particles; ++i) {
    Particle particle;
    particle.pose = uniform_dist(random_generator);
    particle.weight = uniform_weight;
    particle_list->push_back(particle);
  }
}

double EvaluateParticle(const Particle& particle,
                        const std::vector<Position>& observation_list,
                        const GridMap& grid_map) {
  double total_score{0.0};
  int count{0};
  for (const auto& local_position : observation_list) {
    const auto world_position = local_position + particle.pose;
    if (world_position < map_range[0] || world_position > map_range[1])
      continue;
    total_score += InterpolateGridMap(world_position, grid_map);
    ++count;
  }
  if (count < parameters.min_count) return 0.0;
  // total_score /= static_cast<double>(count);
  total_score /= static_cast<double>(100);
  return total_score;
}

double InterpolateGridMap(const Position world_position,
                          const GridMap& grid_map) {
  const auto grid_index =
      ComputeGridMapIndex(world_position, parameters.resolution);
  if (grid_index == grid_map.size() - 1) return grid_map.back();
  const double ratio = (world_position - parameters.resolution * grid_index) /
                       parameters.resolution;
  const double interpolated_grid_value =
      grid_map.at(grid_index) * (1.0 - ratio) +
      grid_map.at(grid_index + 1) * ratio;
  return interpolated_grid_value;
}

void ResampleParticles(const int max_num_particles,
                       std::vector<Particle>* particle_list) {
  std::vector<Particle> resampled_particle_list;
  for (const auto& particle : *particle_list) {
    const int num_duplicates = std::round(particle.weight * max_num_particles);
    for (int index = 0; index < num_duplicates; ++index)
      resampled_particle_list.push_back(particle);
  }

  const double uniform_weight = 1.0 / resampled_particle_list.size();
  for (auto& particle : resampled_particle_list)
    particle.weight = uniform_weight;

  *particle_list = std::move(resampled_particle_list);
}

Pose Localize(const GridMap& grid_map,
              const std::vector<Position>& observation_list,
              std::vector<Particle>* particle_list) {
  cv::Mat image(image_height, image_width, CV_8UC1);
  image.setTo(0);

  Pose previous_estimated_pose{-100.0};
  Pose estimated_pose{0.0};
  for (int iter = 0; iter < parameters.max_iterations; ++iter) {
    // Evaluate particles
    double total_score{0.0};
    for (auto& particle : *particle_list)
      total_score += (particle.weight = EvaluateParticle(
                          particle, observation_list, grid_map));

    for (auto& particle : *particle_list) particle.weight /= total_score;

    // Estimate pose
    estimated_pose = 0.0;
    for (const auto& particle : *particle_list)
      estimated_pose += particle.weight * particle.pose;

    // Resample particles
    ResampleParticles(parameters.max_num_particles, particle_list);

    DrawParticles(*particle_list, estimated_pose, grid_map, &image);

    if (std::abs(previous_estimated_pose - estimated_pose) < 1e-5) break;
    previous_estimated_pose = estimated_pose;
  }
  return estimated_pose;
}

void WarpParticlesWithNoise(const Pose& pose_delta, const double pose_noise_std,
                            std::vector<Particle>* particle_list) {
  std::normal_distribution<double> normal_dist(0.0, pose_noise_std);
  for (auto& particle : *particle_list) {
    particle.pose += pose_delta + normal_dist(random_generator);
    if (particle.pose < map_range[0]) particle.pose = map_range[0];
    if (particle.pose > map_range[1]) particle.pose = map_range[1];
  }
}

void DrawParticles(const std::vector<Particle>& particle_list, const Pose& pose,
                   const GridMap& grid_map, cv::Mat* image) {
  image->setTo(0);
  const int image_height = image->rows;
  const int image_width = image->cols;

  // Draw particles
  for (const auto& particle : particle_list) {
    const int x = static_cast<int>(particle.pose /
                                   (map_range[1] - map_range[0]) * image_width);
    if (x < 0 || x >= image_width) continue;
    for (int y = 350; y < 380; ++y) image->at<uint8_t>(y, x) += 30;
  }

  // Draw pose
  const int x =
      static_cast<int>(pose / (map_range[1] - map_range[0]) * image_width);
  if (x > 0 && x < image_width)
    for (int y = 100; y < 200; ++y) image->at<uint8_t>(y, x) = 255;

  // Draw map
  for (size_t grid_index = 0; grid_index < grid_map.size(); ++grid_index) {
    if (grid_map.at(grid_index) == 0) continue;
    const int x = static_cast<int>(
        grid_index / static_cast<double>(grid_map.size()) * image_width);
    if (x > 0 && x < image_width)
      for (int y = 0; y < 100; ++y) image->at<uint8_t>(y, x) = 255;
  }

  cv::imshow("image", *image);
  cv::waitKey(0);
}