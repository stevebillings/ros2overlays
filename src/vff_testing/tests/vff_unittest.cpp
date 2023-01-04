#include "vff_testing/vff.hpp"
#include <gtest/gtest.h>

TEST(VFF_UNITTEST, ONE) {

  std::vector<float> laser_ranges(2);
  laser_ranges.push_back(10.0);
  laser_ranges.push_back(8.0);
//  laser_ranges.push_back(2.0);
//  laser_ranges.push_back(8.0);
//  laser_ranges.push_back(10.0);

  auto vff = Vff();
  std::vector<float> result = vff.getVff(0.0, 0.1, laser_ranges);
//  std::vector<int> stuff(4);
//  int result = vff.getInt(stuff);

  ASSERT_EQ(result.size(), 2ul);
}