#include "state_search.h"

FsmState StateSearch::act(const History& history, const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const {
  return FsmState::SEARCH;
}

char* StateSearch::name() const {
  return "SEARCH";
}