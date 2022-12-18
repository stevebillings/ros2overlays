#include "state_search.h"

Action StateSearch::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const  {
  return Action(Velocity::create_spin_left(), FsmState::OBSTACLE_NEAR);
}

const char* StateSearch::name() const {
  return "search";
}