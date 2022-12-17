#ifndef OBSTACLE_HUGGER_STATE_NEAR_H
#define OBSTACLE_HUGGER_STATE_NEAR_H

#include "state.h"

class StateNear : public State
{
  Action act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const override;
  const char *name() const;
};


#endif //OBSTACLE_HUGGER_STATE_NEAR_H
