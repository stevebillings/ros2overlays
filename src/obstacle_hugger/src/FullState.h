
#ifndef OBSTACLE_HUGGER_FULLSTATE_H
#define OBSTACLE_HUGGER_FULLSTATE_H

#include "State.h"

class FullState {
public:
    void set_state(State new_state, double state_start_time);
    void set_obstacle_last_seen_time(double obstacle_last_seen_time, bool seen_to_right);
    State get_state();
    const char *get_name();
    double get_state_start_time();
    bool has_obstacle_been_seen();
    double get_obstacle_last_seen_time();
    bool was_obstacle_last_seen_to_right();
private:
    State state_ = State::SEARCH;
    double state_start_time_;
    bool obstacle_has_been_seen_ = false;
    double obstacle_last_seen_time_;
    bool obstacle_last_seen_to_right_;
};

#endif //OBSTACLE_HUGGER_FULLSTATE_H
