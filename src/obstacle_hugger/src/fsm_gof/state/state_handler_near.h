#ifndef OBSTACLE_HUGGER_STATE_HANDLER_NEAR_H
#define OBSTACLE_HUGGER_STATE_HANDLER_NEAR_H

#include "state_handler.h"

class StateHandlerNear : public StateHandler
{
public:
  Action act(const History& history, const double current_time, const LaserCharacteristics& laser_characteristics,
             const LaserAnalysis& laser_analysis) const override;
  const char* name() const;

private:
  const Action handleTooNear() const;
  const Action handleInSight(const LaserAnalysis& laser_analysis) const;
  const Action handleLostSight(const History& history, const LaserAnalysis& laser_analysis) const;
};

#endif  // OBSTACLE_HUGGER_STATE_HANDLER_NEAR_H
