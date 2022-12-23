#include <gtest/gtest.h>
#include "../../../src/fsm_gof/state/state_search.h"

TEST(StateSearchTest, Name)
{
  StateSearch state_search = StateSearch();
  EXPECT_STREQ("search", state_search.name());
}

TEST(StateSearchTest, InSightRight)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 4.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, false, true, 1ul);
  StateSearch state_search = StateSearch();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_search.act(history, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::SEARCH);
  // Ensure the values are reasonable without being overly sensitive to tuning
  EXPECT_TRUE(action.get_velocity().has_value());
  EXPECT_TRUE(action.get_velocity().value().get_forward() > 0.5l);
  EXPECT_TRUE(action.get_velocity().value().get_forward() < 5.0l);
  EXPECT_TRUE(action.get_velocity().value().get_yaw() < 0.0l);
  EXPECT_TRUE(action.get_velocity().value().get_yaw() > -0.1l);
}

TEST(StateSearchTest, Near)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 1.5l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, true, false, true, 1ul);
  StateSearch state_search = StateSearch();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_search.act(history, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::OBSTACLE_NEAR);
}

TEST(StateSearchTest, TooNear)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 1.5l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, true, true, 1ul);
  StateSearch state_search = StateSearch();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_search.act(history, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::OBSTACLE_TOO_NEAR);
}

TEST(StateSearchTest, LostSight)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 20.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, false, false, false, true, 1ul);
  StateSearch state_search = StateSearch();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(5.0l);
  Action action = state_search.act(history, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::SEARCH);
  // Ensure the values are reasonable without being overly sensitive to tuning
  EXPECT_TRUE(action.get_velocity().has_value());
  EXPECT_NEAR(action.get_velocity().value().get_forward(), 0.0l, 0.01L);
  EXPECT_TRUE(action.get_velocity().value().get_yaw() < 0.0l);
  EXPECT_TRUE(action.get_velocity().value().get_yaw() > -1.0l);
}

TEST(StateSearchTest, RecentlyLostSight)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 20.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, false, false, false, true, 1ul);
  StateSearch state_search = StateSearch();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_search.act(history, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::SEARCH);
  EXPECT_FALSE(action.get_velocity().has_value());
}
