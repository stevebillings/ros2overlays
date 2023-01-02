
#ifndef OBSTACLE_HUGGER_STATE_HANDLER_TOO_NEAR_H
#define OBSTACLE_HUGGER_STATE_HANDLER_TOO_NEAR_H

#include "state_handler.h"

class StateHandlerTooNear : public StateHandler
{
public:
  Action act(const History& history, const double current_time, const LaserCharacteristics& laser_characteristics,
             const LaserAnalysis& laser_analysis) const override;
  const char* name() const;
};

#endif  // OBSTACLE_HUGGER_STATE_HANDLER_TOO_NEAR_H
