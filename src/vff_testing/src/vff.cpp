#include "vff_testing/vff.hpp"

[[nodiscard]] std::vector<float> Vff::getVffResult(double laser_angle_min, double laser_angle_increment, std::vector<float>  const & laser_ranges) const {
  const float MINIMUM_IGNORABLE_DISTANCE = 1.0;
  std::vector<float> attractive_vector = {MINIMUM_IGNORABLE_DISTANCE, 0.0}; // Goal: go forward
  std::vector<float> repulsive_vector = {0.0, 0.0};
  std::vector<float> result_vector = {1.0, 0.0};

  int nearest_obstacle_index = std::min_element(laser_ranges.begin(), laser_ranges.end()) - laser_ranges.begin();
  float nearest_obstacle_distance = laser_ranges[nearest_obstacle_index];
  if (nearest_obstacle_distance < MINIMUM_IGNORABLE_DISTANCE) {
    float obstacle_angle = laser_angle_min + laser_angle_increment * nearest_obstacle_index;
    float obstacle_opposite_angle = obstacle_angle + M_PI;
    float repulsive_vector_magnitude = MINIMUM_IGNORABLE_DISTANCE - nearest_obstacle_distance;
    // Calculate cartesian (x, y) components from polar (angle, distance)
    repulsive_vector[0] = cos(obstacle_opposite_angle) * repulsive_vector_magnitude;
    repulsive_vector[1] = sin(obstacle_opposite_angle) * repulsive_vector_magnitude;
  }
  // Add the repulsive vector to the goal vector to get the result vector (path to avoid obstacle)
  result_vector[0] = repulsive_vector[0] + attractive_vector[0];
  result_vector[1] = repulsive_vector[1] + attractive_vector[1];
  return result_vector;
}
