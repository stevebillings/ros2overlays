//
// Created by stevebillings on 12/16/22.
//

#include "state_error.h"

FsmState StateError::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const {
  return FsmState::ERROR;
}

const char* StateError::name() const {
  return "error";
}