#include "state_search.h"

static constexpr double TIME_LOST_TOLERANCE_SECONDS = 0.75;

Action StateSearch::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const  {
  Velocity new_velocity = Velocity::create_stopped();
  FsmState new_state = FsmState::SEARCH;
  if (laser_analysis.isInSight())
    return handleInSight(laser_characteristics, laser_analysis);

  else if (history.has_obstacle_ever_been_seen() && history.get_time_lost() > TIME_LOST_TOLERANCE_SECONDS)
    return handleLostSight(history);

  else if (!history.has_obstacle_ever_been_seen())
    return handleNeverSeen();

  // TODO this is: only recently lost; need a way to return "no change"
  // should we store the last action in history?
  // add a boolean to action?
  // subclass Action? Or Velocity?
  // use null fields in Action object to signify no change? This actually seems problematic for doubles.
  // for state, could add another enum value
  // velocity could have a no change boolean
  // In Action, make velocity std::optional<Velocity>
  return Action(new_velocity, new_state);
}

const char* StateSearch::name() const {
  return "search";
}

Action StateSearch::handleInSight(const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const {
  Velocity new_velocity = Velocity::create_stopped();
  FsmState new_state = FsmState::SEARCH;
  new_velocity = velocity_calculator_.toApproach(laser_characteristics, laser_analysis);
  if (laser_analysis.isNear())
  {
    new_state = FsmState::OBSTACLE_NEAR;
  }
  return Action(new_velocity, new_state);
}

Action StateSearch::handleLostSight(const History& history) const {
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

Action StateSearch::handleNeverSeen() const {
  Velocity new_velocity = Velocity::create_stopped();
  FsmState new_state = FsmState::SEARCH;
  new_velocity = Velocity::create_spin_right();
  return Action(new_velocity, new_state);
}