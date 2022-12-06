
#ifndef OBSTACLE_HUGGER_FULLSTATE_H
#define OBSTACLE_HUGGER_FULLSTATE_H

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "FsmState.h"

// times are expressed in seconds since epoch
class FullState
{
public:
  void set_state(const FsmState& new_state, double state_start_time);
  void set_obstacle_last_seen_time(const rclcpp::Logger& logger, double obstacle_last_seen_time, bool seen_to_right);
  FsmState get_fsm_state() const;
  const char* get_fsm_state_name() const;
  double get_state_start_time() const;
  bool has_obstacle_been_seen() const;
  double get_obstacle_last_seen_time() const;
  bool was_obstacle_last_seen_to_right() const;

private:
  FsmState fsm_state_ = FsmState::SEARCH;
  double state_start_time_ = 0.0;
  bool obstacle_has_been_seen_ = false;
  double obstacle_last_seen_time_ = 0.0;
  bool obstacle_last_seen_to_right_ = false;
};

#endif  // OBSTACLE_HUGGER_FULLSTATE_H
