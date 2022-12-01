
#ifndef OBSTACLE_HUGGER_FULLSTATE_H
#define OBSTACLE_HUGGER_FULLSTATE_H

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "FsmState.h"

// times are expressed in seconds since epoch
class FullState {
public:
    FullState() : fsm_state_(FsmState::SEARCH)
        , state_start_time_(0.0)
        , obstacle_has_been_seen_(false)
        , obstacle_last_seen_time_(0.0)
        , obstacle_last_seen_to_right_(false) {}
    void set_state(FsmState new_state, double state_start_time);
    void set_obstacle_last_seen_time(rclcpp::Logger& logger, double obstacle_last_seen_time, bool seen_to_right);
    FsmState get_fsm_state();
    const char *get_fsm_state_name();
    double get_state_start_time();
    bool has_obstacle_been_seen();
    double get_obstacle_last_seen_time();
    bool was_obstacle_last_seen_to_right();
private:
    FsmState fsm_state_ = FsmState::SEARCH;
    double state_start_time_;
    bool obstacle_has_been_seen_ = false;
    double obstacle_last_seen_time_;
    bool obstacle_last_seen_to_right_;
};

#endif //OBSTACLE_HUGGER_FULLSTATE_H
