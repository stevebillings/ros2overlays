#include "vff_testing/vff.hpp"

[[nodiscard]] std::vector<float> Vff::getVff(double laser_angle_min, double laser_angle_increment, std::vector<float>  const & laser_ranges) const {
  std::vector<float> result = { 1.1, 2.2};
  return result;
}

int Vff::getInt(std::vector<int> & stuff) {
  return 1;
}