//
// Created by stevebillings on 12/16/22.
//

#ifndef OBSTACLE_HUGGER_STATE_HANDLERS_H
#define OBSTACLE_HUGGER_STATE_HANDLERS_H

#include "state/state_handler.h"
#include "state/state_handler_search.h"
#include "state/state_handler_near.h"
#include "state/state_handler_too_near.h"
#include "state/state_handler_error.h"

// this class constructs and holds all state objects, and has a method that returns the state object for a given
// fsm_state enum value (state objects must be stateless)
class StateHandlers
{
public:
  StateHandler* get_state_handler(FsmState state) const;

private:
  StateHandler* state_handler_search_ = new StateHandlerSearch();
  StateHandler* state_handler_near_ = new StateHandlerNear();
  StateHandler* state_handler_too_near_ = new StateHandlerTooNear();
  StateHandler* state_handler_error_ = new StateHandlerError();
};

#endif  // OBSTACLE_HUGGER_STATE_HANDLERS_H
