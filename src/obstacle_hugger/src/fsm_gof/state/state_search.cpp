#include "state_search.h"

static constexpr double TIME_LOST_TOLERANCE_SECONDS = 0.75;

Action StateSearch::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const  {
  Velocity new_velocity = Velocity::create_stopped();
  FsmState new_state = FsmState::SEARCH;
  if (laser_analysis.isInSight())
  {
    new_velocity = velocity_calculator_.toApproach(laser_characteristics, laser_analysis);
//    RCLCPP_INFO(logger_, "Approaching: x: %lf; yaw: %lf", approach_velocity.get_forward(),
//                approach_velocity.get_yaw());
    if (laser_analysis.isNear())
    {
      new_state = FsmState::OBSTACLE_NEAR;
    }
  }
  else if (history.has_obstacle_ever_been_seen() && !laser_analysis.isInSight()
           && history.get_time_lost() > TIME_LOST_TOLERANCE_SECONDS)
  {
//    RCLCPP_WARN(logger_, "We've lost track of the obstacle for more than %lf seconds", TIME_LOST_TOLERANCE_SECONDS);
    if (history.was_seen_to_right())
    {
      new_velocity = Velocity::create_spin_right();
    }
    else
    {
      new_velocity = Velocity::create_spin_left();
    }
  }
  else if (!history.has_obstacle_ever_been_seen())
  {
    new_velocity = Velocity::create_spin_right();
  }
  return Action(new_velocity, new_state);
}

const char* StateSearch::name() const {
  return "search";
}