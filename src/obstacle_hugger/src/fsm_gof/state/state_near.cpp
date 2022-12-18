
#include "state_near.h"

Action StateNear::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const  {
  return Action(Velocity(1.0l, 0.0l), FsmState::OBSTACLE_TOO_NEAR);
}

const char* StateNear::name() const {
  return "obstacle near";
}