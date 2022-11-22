//
// Created by stevebillings on 11/21/22.
//

#include "LaserAnalysis.h"

LaserAnalysis::LaserAnalysis(unsigned long min_range_index, double min_range, bool in_sight, bool near,
                             unsigned long leftmost_index) {
    min_range_index_ = min_range_index;
    min_range_ = min_range;
    in_sight_ = in_sight;
    near_ = near;
    leftmost_index_ = leftmost_index;
}
unsigned long LaserAnalysis::get_min_range_index() {
    return min_range_index_;
}
double LaserAnalysis::get_min_range() {
    return min_range_;
}

bool LaserAnalysis::is_in_sight() {
    return in_sight_;
}

bool LaserAnalysis::is_near() {
    return near_;
}

unsigned long LaserAnalysis::get_leftmost_index() {
    return leftmost_index_;
}
