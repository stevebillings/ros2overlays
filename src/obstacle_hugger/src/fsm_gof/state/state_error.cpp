
#include "state_error.h"

Action StateError::act(const History &history, const double current_time, const LaserCharacteristics &laser_characteristics,
                       const LaserAnalysis &laser_analysis) const
{
  return Action(Velocity::create_stopped(), FsmState::ERROR);
}

const char *StateError::name() const
{
  return "error";
}