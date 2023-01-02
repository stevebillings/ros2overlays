#include "state_handler_too_near.h"

Action StateHandlerTooNear::act(const History& history, const double current_time,
                                const LaserCharacteristics& laser_characteristics,
                                const LaserAnalysis& laser_analysis) const
{
  if ((current_time - history.get_time_entered_state()) > 2.0)
  {
    return Action(Velocity::create_stopped(), FsmState::SEARCH);
  }
  return Action(FsmState::OBSTACLE_TOO_NEAR);
}

const char* StateHandlerTooNear::name() const
{
  return "obstacle too near";
}