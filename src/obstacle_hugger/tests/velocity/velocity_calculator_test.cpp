#include <gtest/gtest.h>
#include "../../src/velocity/velocity_calculator.h"

TEST(VelocityCalculatorTest, AheadRight)
{
  VelocityCalculator velocity_calculator;
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 3.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, false, true, 1ul);
  Velocity velocity = velocity_calculator.toApproach(laser_characteristics, laser_analysis);
  EXPECT_NEAR(velocity.get_forward(), 0.75l, .05l);
  EXPECT_NEAR(velocity.get_yaw(), -0.01l, .001l);
}

TEST(VelocityCalculatorTest, FarAheadLeft)
{
  VelocityCalculator velocity_calculator;
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(3ul, 9.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, false, true, 1ul);
  Velocity velocity = velocity_calculator.toApproach(laser_characteristics, laser_analysis);
  EXPECT_NEAR(velocity.get_forward(), 2.25l, .05l);
  EXPECT_NEAR(velocity.get_yaw(), 0.01l, .001l);
}