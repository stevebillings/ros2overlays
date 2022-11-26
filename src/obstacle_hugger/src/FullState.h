
#ifndef OBSTACLE_HUGGER_FULLSTATE_H
#define OBSTACLE_HUGGER_FULLSTATE_H

#include "State.h"

class FullState {
public:
    void set_state(State new_state, long state_start_time);
    void set_obstacle_last_seen_time(long obstacle_last_seen_time, bool seen_to_right);
    State get_state();
    char *get_name();
    long get_state_start_time();
    bool has_obstacle_been_seen();
    long get_obstacle_last_seen_time();
    bool was_obstacle_last_seen_to_right();
private:
    State state_ = State::SEARCH;
    long state_start_time_;
    bool obstacle_has_been_seen_ = false;
    long obstacle_last_seen_time_;
    bool obstacle_last_seen_to_right_;
};

#endif //OBSTACLE_HUGGER_FULLSTATE_H
