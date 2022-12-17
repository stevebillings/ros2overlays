//
// Created by stevebillings on 12/16/22.
//

#include "state_too_near.h"

FsmState StateTooNear::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const {
  return FsmState::ERROR;
}

const char* StateTooNear::name() const {
  return "obstacle too near";
}