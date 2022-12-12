#include <gtest/gtest.h>
#include "../src/laser/laser_analyzer.h"

TEST(LaserTest, CrazySimple) {
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(7*6, 42);
}

TEST(LaserTest, FirstTest) {
  LaserAnalyzer laserAnalyzer;
  laserAnalyzer.analyze();
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(7*6, 42);
}