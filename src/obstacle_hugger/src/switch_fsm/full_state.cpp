
#include "full_state.h"

void FullState::setState(const FsmState& new_state, double state_start_time)
{
  fsm_state_ = new_state;
  state_start_time_ = state_start_time;
}
void FullState::setObstacleLastSeenTime(const rclcpp::Logger& logger, double obstacle_last_seen_time,
                                        bool seen_to_right)
{
  RCLCPP_INFO(logger, "Obstacle seen at time %lf\n", obstacle_last_seen_time);
  obstacle_has_been_seen_ = true;
  obstacle_last_seen_time_ = obstacle_last_seen_time;
  obstacle_last_seen_to_right_ = seen_to_right;
}

FsmState FullState::getFsmState() const
{
  return fsm_state_;
}

const char* FullState::getFsmStateName() const
{
  switch (fsm_state_)
  {
    case FsmState::SEARCH:
      return "SEARCH";
    case FsmState::OBSTACLE_NEAR:
      return "OBSTACLE_NEAR";
    case FsmState::OBSTACLE_TOO_NEAR:
      return "OBSTACLE_TOO_NEAR";
    case FsmState::ERROR:
      return "ERROR";
    default:
      return "UNRECOGNIZED";
  }
}

bool FullState::hasObstacleBeenSeen() const
{
  return obstacle_has_been_seen_;
}
double FullState::getObstacleLastSeenTime() const
{
  return obstacle_last_seen_time_;
}
bool FullState::wasObstacleLastSeenToRight() const
{
  return obstacle_last_seen_to_right_;
}

double FullState::getStateStartTime() const
{
  return state_start_time_;
}
