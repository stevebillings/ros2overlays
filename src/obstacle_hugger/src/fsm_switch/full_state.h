
#ifndef OBSTACLE_HUGGER_FULL_STATE_H
#define OBSTACLE_HUGGER_FULL_STATE_H

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "../common/fsm_state.h"

// times are expressed in seconds since epoch
class FullState
{
public:
  void setState(const FsmState& new_state, double state_start_time);
  void setObstacleLastSeenTime(double obstacle_last_seen_time, bool seen_to_right);
  FsmState getFsmState() const;
  const char* getFsmStateName() const;
  double getStateStartTime() const;
  bool hasObstacleBeenSeen() const;
  double getObstacleLastSeenTime() const;
  bool wasObstacleLastSeenToRight() const;

private:
  FsmState fsm_state_ = FsmState::SEARCH;
  double state_start_time_ = 0.0;
  bool obstacle_has_been_seen_ = false;
  double obstacle_last_seen_time_ = 0.0;
  bool obstacle_last_seen_to_right_ = false;
};

#endif  // OBSTACLE_HUGGER_FULL_STATE_H
