#include "state_handler_search.h"

static constexpr double TIME_LOST_TOLERANCE_SECONDS = 0.75;

Action StateHandlerSearch::act(const History& history, const double current_time,
                               const LaserCharacteristics& laser_characteristics,
                               const LaserAnalysis& laser_analysis) const
{
  if (laser_analysis.isInSight())
    return handleInSight(laser_characteristics, laser_analysis);
  else if (history.has_obstacle_ever_been_seen() && history.get_time_lost() > TIME_LOST_TOLERANCE_SECONDS)
    return handleLostSight(history);
  else if (!history.has_obstacle_ever_been_seen())
    return handleNeverSeen();
  else
    return handleRecentlyLost();
}

const char* StateHandlerSearch::name() const
{
  return "search";
}

Action StateHandlerSearch::handleRecentlyLost() const
{
  return Action(FsmState::SEARCH);
}

Action StateHandlerSearch::handleInSight(const LaserCharacteristics& laser_characteristics,
                                         const LaserAnalysis& laser_analysis) const
{
  FsmState new_state = FsmState::SEARCH;
  Velocity new_velocity = velocity_calculator_.toApproach(laser_characteristics, laser_analysis);
  if (laser_analysis.isNear())
    new_state = FsmState::OBSTACLE_NEAR;
  else if (laser_analysis.isTooNear())
    new_state = FsmState::OBSTACLE_TOO_NEAR;
  return Action(new_velocity, new_state);
}

Action StateHandlerSearch::handleLostSight(const History& history) const
{
  Velocity new_velocity = Velocity::create_stopped();
  FsmState new_state = FsmState::SEARCH;
  if (history.was_seen_to_right())
  {
    new_velocity = Velocity::create_spin_right();
  }
  else
  {
    new_velocity = Velocity::create_spin_left();
  }
  return Action(new_velocity, new_state);
}

Action StateHandlerSearch::handleNeverSeen() const
{
  Velocity new_velocity = Velocity::create_stopped();
  FsmState new_state = FsmState::SEARCH;
  new_velocity = Velocity::create_spin_right();
  return Action(new_velocity, new_state);
}