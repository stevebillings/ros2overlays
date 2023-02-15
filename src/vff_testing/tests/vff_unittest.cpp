#include "vff_testing/vff.hpp"
#include <gtest/gtest.h>

const double LASER_ANGLE_MINIMUM = -1.396263;
const double LASER_ANGLE_INCREMENT = 0.004370;


TEST(VFF_UNITTEST, FAR) {
  std::vector<float> laser_ranges(640);
  for (int i=0; i<640; i++) {
    laser_ranges[i] = 10.0;
  }

  auto vff = Vff();
  std::vector<float> result = vff.getVffResult(LASER_ANGLE_MINIMUM, LASER_ANGLE_INCREMENT, laser_ranges);

  ASSERT_EQ(result.size(), 2ul);
  ASSERT_NEAR(result[0], 1.0, 0.01);
  ASSERT_NEAR(result[1], 0.0, 0.001);
}

TEST(VFF_UNITTEST, AHEAD_CLOSE) {
  std::vector<float> laser_ranges(640);
  for (int i=0; i<640; i++) {
    laser_ranges[i] = 10.0;
  }
  laser_ranges[319] = 0.5;

  auto vff = Vff();
  std::vector<float> result = vff.getVffResult(LASER_ANGLE_MINIMUM, LASER_ANGLE_INCREMENT, laser_ranges);

  ASSERT_EQ(result.size(), 2ul);
  ASSERT_NEAR(result[0], 0.5, 0.1);
  ASSERT_NEAR(result[1], 0.0, 0.1);
}

TEST(VFF_UNITTEST, RIGHT_CLOSE) {
  std::vector<float> laser_ranges(640);
  for (int i=0; i<640; i++) {
    laser_ranges[i] = 10.0;
  }
  laser_ranges[159] = 0.1;

  auto vff = Vff();
  std::vector<float> result = vff.getVffResult(LASER_ANGLE_MINIMUM, LASER_ANGLE_INCREMENT, laser_ranges);

  ASSERT_EQ(result.size(), 2ul);
  ASSERT_NEAR(result[0], 0.3, 0.1);
  ASSERT_NEAR(result[1], 0.6, 0.1);
}
