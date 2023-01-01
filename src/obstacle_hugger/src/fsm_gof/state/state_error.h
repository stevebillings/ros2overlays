
#ifndef OBSTACLE_HUGGER_STATE_ERROR_H
#define OBSTACLE_HUGGER_STATE_ERROR_H

#include "state.h"

class StateError : public State
{
  Action act(const History& history, const double current_time, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const override;
  const char *name() const;
};



#endif //OBSTACLE_HUGGER_STATE_ERROR_H
