
#include "state_near.h"

Action StateNear::act(const History &history, const LaserCharacteristics &laser_characteristics,
                      const LaserAnalysis &laser_analysis) const
{
  if (laser_analysis.isTooNear())
    return handleTooNear();
  else if (laser_analysis.isInSight())
    return handleInSight(laser_analysis);
  else
    return handleLostSight(history, laser_analysis);
}

const char *StateNear::name() const
{
  return "obstacle near";
}

const Action StateNear::handleTooNear() const
{
  return Action(Velocity::create_reverse(), FsmState::OBSTACLE_TOO_NEAR);
}

const Action StateNear::handleInSight(const LaserAnalysis &laser_analysis) const
{
  return Action(velocity_calculator_.toParallel(laser_analysis), FsmState::OBSTACLE_NEAR);
}

const Action StateNear::handleLostSight(const History &history, const LaserAnalysis &laser_analysis) const
{
  if (history.get_time_lost() > 1.0)
    return Action(Velocity::create_stopped(), FsmState::SEARCH);
  else
    return Action(FsmState::OBSTACLE_NEAR);
}