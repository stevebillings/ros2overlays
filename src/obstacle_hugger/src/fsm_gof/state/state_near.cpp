
#include "state_near.h"

FsmState StateNear::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const {
  return FsmState::OBSTACLE_TOO_NEAR;
}

const char* StateNear::name() const {
  return "obstacle near";
}