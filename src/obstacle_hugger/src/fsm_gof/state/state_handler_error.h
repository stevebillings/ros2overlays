
#ifndef OBSTACLE_HUGGER_STATE_HANDLER_ERROR_H
#define OBSTACLE_HUGGER_STATE_HANDLER_ERROR_H

#include "state_handler.h"

class StateHandlerError : public StateHandler
{
  Action act(const History& history, const double current_time, const LaserCharacteristics& laser_characteristics,
             const LaserAnalysis& laser_analysis) const override;
  const char* name() const;
};

#endif  // OBSTACLE_HUGGER_STATE_HANDLER_ERROR_H
