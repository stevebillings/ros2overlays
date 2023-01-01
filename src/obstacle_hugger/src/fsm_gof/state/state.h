#ifndef OBSTACLE_HUGGER_STATE_H
#define OBSTACLE_HUGGER_STATE_H

#include "../../common/fsm_state.h"
#include "../../laser/laser_characteristics.h"
#include "../../laser/laser_analysis.h"
#include "../../history/history.h"
#include "../action.h"
#include "../../velocity/velocity_calculator.h"

// TODO this is a state handler, not a state

// the virtual class for State; each concrete state object implement the act() method for a specific state
class State
{
public:
  virtual const char* name() const = 0;
  virtual Action act(const History& history, const double current_time, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const = 0;
  virtual ~State() {};
protected:
  VelocityCalculator velocity_calculator_ = VelocityCalculator();
};


#endif //OBSTACLE_HUGGER_STATE_H
