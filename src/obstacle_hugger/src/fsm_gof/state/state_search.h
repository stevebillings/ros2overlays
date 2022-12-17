
#ifndef OBSTACLE_HUGGER_STATE_SEARCH_H
#define OBSTACLE_HUGGER_STATE_SEARCH_H

#include "state.h"

class StateSearch : public State
{
  Action act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const override;
  const char *name() const;
};


#endif //OBSTACLE_HUGGER_STATE_SEARCH_H
