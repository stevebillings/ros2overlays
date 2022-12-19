#include <gtest/gtest.h>
#include "../../../src/fsm_gof/state/state_search.h"

TEST(StateSearchTest, Name)
{
  StateSearch state_search = StateSearch();
  EXPECT_STREQ("search", state_search.name());
}

TEST(StateSearchTest, InSightRight)
{
  VelocityCalculator velocity_calculator;
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 4.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, false, true, 1ul);
  StateSearch state_search = StateSearch();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_search.act(history, laser_characteristics, laser_analysis);
  // Ensure the values are reasonable without being overly sensitive to tuning
  EXPECT_TRUE(action.get_velocity().get_forward() > 0.5l);
  EXPECT_TRUE(action.get_velocity().get_forward() < 5.0l);
  EXPECT_TRUE(action.get_velocity().get_yaw() < 0.0l);
  EXPECT_TRUE(action.get_velocity().get_yaw() > -0.1l);
}
