
#include "state_near.h"

Action StateNear::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const  {
  return Action(Velocity::createSearchSpinRight(), FsmState::OBSTACLE_TOO_NEAR);
}

const char* StateNear::name() const {
  return "obstacle near";
}