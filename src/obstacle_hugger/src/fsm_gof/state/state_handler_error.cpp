
#include "state_handler_error.h"

Action StateHandlerError::act(const History& history, const double current_time,
                              const LaserCharacteristics& laser_characteristics,
                              const LaserAnalysis& laser_analysis) const
{
  return Action(Velocity::create_stopped(), FsmState::ERROR);
}

const char* StateHandlerError::name() const
{
  return "error";
}