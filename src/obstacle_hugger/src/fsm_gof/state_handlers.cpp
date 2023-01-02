//
// Created by stevebillings on 12/16/22.
//

#include "state_handlers.h"

StateHandler* StateHandlers::get_state_handler(FsmState state) const
{
  switch (state)
  {
    case FsmState::SEARCH:
      return state_handler_search_;
    case FsmState::OBSTACLE_NEAR:
      return state_handler_near_;
    case FsmState::OBSTACLE_TOO_NEAR:
      return state_handler_too_near_;
    default:
      return state_handler_error_;
  }
}