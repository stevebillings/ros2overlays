#ifndef OBSTACLE_HUGGER_STATE_H
#define OBSTACLE_HUGGER_STATE_H

#include "../../common/fsm_state.h"
#include "../../laser/laser_characteristics.h"
#include "../../laser/laser_analysis.h"
#include "../../history/history.h"

// the virtual class for State; each concrete state object implement the act() method for a specific state
class State
{
public:
  virtual const char* name() const = 0;
  virtual FsmState act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const = 0;
};


#endif //OBSTACLE_HUGGER_STATE_H
