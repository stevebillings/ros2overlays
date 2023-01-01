#include "state_too_near.h"

Action StateTooNear::act(const History &history, const LaserCharacteristics &laser_characteristics,
                         const LaserAnalysis &laser_analysis) const
{
  return Action(Velocity::create_reverse(), FsmState::ERROR);
}

const char *StateTooNear::name() const
{
  return "obstacle too near";
}