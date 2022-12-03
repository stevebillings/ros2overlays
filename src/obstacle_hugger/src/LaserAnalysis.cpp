//
// Created by stevebillings on 11/21/22.
//

#include "LaserAnalysis.h"

LaserAnalysis::LaserAnalysis(unsigned long min_range_index, double min_range, bool in_sight, bool near, bool too_near,
                             unsigned long leftmost_index,
                             unsigned long straight_index,
                             bool to_right,
                             unsigned long delta_from_perpendicular_right,
                             unsigned long delta_from_perpendicular_left) {
    min_range_index_ = min_range_index;
    min_range_ = min_range;
    in_sight_ = in_sight;
    near_ = near;
    too_near_ = too_near;
    leftmost_index_ = leftmost_index;
    straight_index_ = straight_index;
    to_right_ = to_right;
    delta_from_perpendicular_right_ = delta_from_perpendicular_right;
    delta_from_perpendicular_left_ = delta_from_perpendicular_left;
}
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
