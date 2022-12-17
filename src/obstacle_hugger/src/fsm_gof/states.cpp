//
// Created by stevebillings on 12/16/22.
//

#include "states.h"

State* States::get_state(FsmState state) const {
  return state_search_;
}