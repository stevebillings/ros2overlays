//
// Created by stevebillings on 11/21/22.
//

#include "LaserAnalysis.h"

unsigned long LaserAnalysis::get_min_range_index() const {
    return min_range_index_;
}
double LaserAnalysis::get_min_range() const {
    return min_range_;
}

bool LaserAnalysis::is_in_sight() const {
    return in_sight_;
}

bool LaserAnalysis::is_near() const {
    return near_;
}

bool LaserAnalysis::is_too_near() const {
    return too_near_;
}

unsigned long LaserAnalysis::get_leftmost_index() const {
    return leftmost_index_;
}

unsigned long LaserAnalysis::get_straight_index() const {
    return straight_index_;
}

bool LaserAnalysis::is_to_right() const {
    return to_right_;
}

unsigned long LaserAnalysis::get_delta_from_perpendicular_right() const {
    return delta_from_perpendicular_right_;
}
unsigned long LaserAnalysis::get_delta_from_perpendicular_left() const {
    return delta_from_perpendicular_left_;
}
