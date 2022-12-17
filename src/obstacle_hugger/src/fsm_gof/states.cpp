//
// Created by stevebillings on 12/16/22.
//

#include "states.h"

State* States::get_state(FsmState state) const {
  switch (state) {
    case FsmState::SEARCH:
      return state_search_;
    case FsmState::OBSTACLE_NEAR:
      return state_near_;
    case FsmState::OBSTACLE_TOO_NEAR:
      return state_too_near_;
    default:
      return state_error_;
  }
}