#include "state_search.h"

FsmState StateSearch::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const {
  return FsmState::OBSTACLE_NEAR;
}

const char* StateSearch::name() const {
  return "search";
}