#include <gtest/gtest.h>
#include "../../src/laser/laser_analyzer.h"

TEST(LaserTest, CharacteristicsTest)
{
  LaserAnalyzer laserAnalyzer;
  std::vector<float> laser_ranges;
  laser_ranges.push_back(10.0);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(2.0);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(10.0);

  LaserCharacteristics laser_characteristics = laserAnalyzer.determineCharacteristics(laser_ranges);
  EXPECT_EQ(laser_characteristics.getLeftmostIndex(), 4ul);
  EXPECT_EQ(laser_characteristics.getStraightIndex(), 2ul);
}

TEST(LaserTest, AnalysisTest)
{
  LaserAnalyzer laserAnalyzer;
  std::vector<float> laser_ranges;
  laser_ranges.push_back(10.0);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(1.5);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(10.0);

  LaserCharacteristics laser_characteristics = laserAnalyzer.determineCharacteristics(laser_ranges);
  LaserAnalysis laser_analysis = laserAnalyzer.analyze(laser_characteristics, laser_ranges);
  EXPECT_TRUE(laser_analysis.isInSight());
  NearestSighting nearest_sighting = laser_analysis.getNearestSighting();
  EXPECT_EQ(nearest_sighting.getRangeIndex(), 2ul);
  EXPECT_EQ(nearest_sighting.getRange(), 1.5);
  EXPECT_TRUE(laser_analysis.isNear());
  EXPECT_FALSE(laser_analysis.isTooNear());
  // EXPECT_TRUE(laser_analysis.isToRight());
  EXPECT_EQ(laser_analysis.getDeltaFromPerpendicular(), 2ul);
}

TEST(LaserTest, AnalysisSideTest)
{
  LaserAnalyzer laserAnalyzer;
  std::vector<float> laser_ranges;
  laser_ranges.push_back(3.0);
  laser_ranges.push_back(1.6);
  laser_ranges.push_back(6.0);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(10.0);

  LaserCharacteristics laser_characteristics = laserAnalyzer.determineCharacteristics(laser_ranges);
  LaserAnalysis laser_analysis = laserAnalyzer.analyze(laser_characteristics, laser_ranges);
  EXPECT_TRUE(laser_analysis.isInSight());
  NearestSighting nearest_sighting = laser_analysis.getNearestSighting();
  EXPECT_EQ(nearest_sighting.getRangeIndex(), 1ul);
  EXPECT_NEAR(nearest_sighting.getRange(), 1.6, 0.001);
  EXPECT_TRUE(laser_analysis.isNear());
  EXPECT_FALSE(laser_analysis.isTooNear());
  EXPECT_TRUE(laser_analysis.isToRight());
  EXPECT_EQ(laser_analysis.getDeltaFromPerpendicular(), 1ul);
}

TEST(LaserTest, AnalysisTooNearTest)
{
  LaserAnalyzer laserAnalyzer;
  std::vector<float> laser_ranges;
  laser_ranges.push_back(3.0);
  laser_ranges.push_back(1.4);
  laser_ranges.push_back(6.0);
  laser_ranges.push_back(8.0);
  laser_ranges.push_back(10.0);

  LaserCharacteristics laser_characteristics = laserAnalyzer.determineCharacteristics(laser_ranges);
  LaserAnalysis laser_analysis = laserAnalyzer.analyze(laser_characteristics, laser_ranges);
  EXPECT_TRUE(laser_analysis.isTooNear());
}