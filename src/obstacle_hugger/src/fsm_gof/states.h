//
// Created by stevebillings on 12/16/22.
//

#ifndef OBSTACLE_HUGGER_STATES_H
#define OBSTACLE_HUGGER_STATES_H

#include "state/state.h"
#include "state/state_search.h"
#include "state/state_near.h"
#include "state/state_too_near.h"
#include "state/state_error.h"

// this class constructs and holds all state objects, and has a method that returns the state object for a given fsm_state enum value
// (state objects must be stateless)
class States
{
public:
  State* get_state(FsmState state) const;
private:
  State* state_search_ = new StateSearch();
  State* state_near_ = new StateNear();
  State* state_too_near_ = new StateTooNear();
  State* state_error_ = new StateError();
};


#endif //OBSTACLE_HUGGER_STATES_H
