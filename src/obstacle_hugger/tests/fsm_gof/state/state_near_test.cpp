#include <gtest/gtest.h>
#include "../../../src/fsm_gof/state/state_handler_near.h"

TEST(StateNearTest, Name)
{
  StateHandlerNear state_near = StateHandlerNear();
  EXPECT_STREQ("obstacle near", state_near.name());
}

TEST(StateNearTest, InSightFarRight)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 4.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, false, true, 1ul);
  StateHandlerNear state_near = StateHandlerNear();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_near.act(history, 0.0l, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::OBSTACLE_NEAR);
  // Ensure the values are reasonable without being overly sensitive to magnitude
  EXPECT_TRUE(action.get_velocity().has_value());
  EXPECT_TRUE(action.get_velocity().value().get_forward() > 0.5l);
  EXPECT_TRUE(action.get_velocity().value().get_forward() < 5.0l);
}

TEST(StateNearTest, Near)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 1.5l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, true, false, true, 1ul);
  StateHandlerNear state_near = StateHandlerNear();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_near.act(history, 0.0l, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::OBSTACLE_NEAR);
  EXPECT_TRUE(action.get_velocity().has_value());
  EXPECT_NEAR(action.get_velocity().value().get_yaw(), 0.01l, 0.001L);
  EXPECT_TRUE(action.get_velocity().value().get_forward() >= 1.0l);
}

TEST(StateNearTest, TooNear)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 0.5l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, true, true, 1ul);
  StateHandlerNear state_near = StateHandlerNear();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_near.act(history, 0.0l, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::OBSTACLE_TOO_NEAR);
  EXPECT_TRUE(action.get_velocity().has_value());
  EXPECT_NEAR(action.get_velocity().value().get_forward(), -1.0l, 0.01L);
  EXPECT_NEAR(action.get_velocity().value().get_yaw(), 0.0l, 0.01L);
}

TEST(StateNearTest, JustLostSight)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 100.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, false, false, false, true, 1ul);
  StateHandlerNear state_near = StateHandlerNear();
  History history = History();
  history.set_obstacle_last_seen_time(0.0l, true);
  history.set_time_lost(0.1l);
  Action action = state_near.act(history, 0.0l, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::OBSTACLE_NEAR);
  EXPECT_FALSE(action.get_velocity().has_value());
}

TEST(StateNearTest, LostSightLongAgo)
{
  LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
  NearestSighting nearest_sighting = NearestSighting(1ul, 20.0l);
  LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, false, false, false, true, 1ul);
  StateHandlerNear state_near = StateHandlerNear();
  History history = History();
  history.set_obstacle_last_seen_time(1000.0l, true);
  history.set_time_lost(5.0l);
  Action action = state_near.act(history, 0.0l, laser_characteristics, laser_analysis);
  EXPECT_EQ(action.get_state(), FsmState::SEARCH);
  // Ensure the values are reasonable without being overly sensitive to magnitude
  EXPECT_TRUE(action.get_velocity().has_value());
  EXPECT_NEAR(action.get_velocity().value().get_forward(), 0.0l, 0.01L);
  EXPECT_NEAR(action.get_velocity().value().get_yaw(), 0.0l, 0.01L);
}

