//
// Created by stevebillings on 12/16/22.
//

#ifndef OBSTACLE_HUGGER_STATE_TOO_NEAR_H
#define OBSTACLE_HUGGER_STATE_TOO_NEAR_H

#include "state.h"

class StateTooNear : public State
{
  FsmState act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const;
  const char *name() const;
};



#endif //OBSTACLE_HUGGER_STATE_TOO_NEAR_H
