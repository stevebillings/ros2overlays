#include <stdio.h>
#include "FullState.h"

void FullState::set_state(State new_state, double state_start_time) {
    state_ = new_state;
    state_start_time_ = state_start_time;
}
void FullState::set_obstacle_last_seen_time(double obstacle_last_seen_time, bool seen_to_right) {
    printf("Obstacle seen at %lf\n", obstacle_last_seen_time);
    obstacle_has_been_seen_ = true;
    obstacle_last_seen_time_ = obstacle_last_seen_time;
    obstacle_last_seen_to_right_ = seen_to_right;
}

State FullState::get_state() {
    return state_;
}

const char *FullState::get_name() {
    switch (state_) {
        case SEARCH:
            return "SEARCH";
        case OBSTACLE_NEAR:
            return "OBSTACLE_NEAR";
        case OBSTACLE_TOO_NEAR:
            return "OBSTACLE_TOO_NEAR";
        case RECENTLY_LOST:
            return "RECENTLY_LOST";
        default:
            return "UNRECOGNIZED";
    }
}

bool FullState::has_obstacle_been_seen() {
    return obstacle_has_been_seen_;
}
double FullState::get_obstacle_last_seen_time() {
    return obstacle_last_seen_time_;
}
bool FullState::was_obstacle_last_seen_to_right() {
    return obstacle_last_seen_to_right_;
}

double FullState::get_state_start_time() {
    return state_start_time_;
}
