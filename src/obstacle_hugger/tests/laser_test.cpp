#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../src/laser/laser_analyzer.h"

class MockSimpleLogger {
public:
  MOCK_METHOD(void, log, (const char* msg), (const));
};


TEST(LaserTest, CrazySimple) {
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(7*6, 42);
}

TEST(LaserTest, FirstTest) {
  MockSimpleLogger logger;
  LaserAnalyzer laserAnalyzer;
  std::vector<float> laser_ranges;
  laser_ranges.push_back(10.0);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(2.0);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(10.0);

  LaserCharacteristics laser_characteristics = laserAnalyzer.determineCharacteristics(laser_ranges);
  EXPECT_EQ(laser_characteristics.getLeftmostIndex(), 4ul);
}