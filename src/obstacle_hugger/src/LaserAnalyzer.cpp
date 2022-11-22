#include "LaserAnalyzer.h"

LaserAnalyzer::LaserAnalyzer() {}

LaserAnalysis LaserAnalyzer::analyze(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    unsigned long straight_index = msg->ranges.size() / 2;
    int cur_range_index = 0;
    double min_range = 1000.0;
    unsigned long min_range_index = straight_index;
    for (auto this_range : msg->ranges) {
        if (this_range < min_range) {
            min_range = this_range;
            min_range_index = cur_range_index;
        }
        cur_range_index++;
    }
    bool in_sight = min_range < DIST_WITHIN_SIGHT;
    bool near = min_range < DIST_NEAR;
    unsigned long leftmost_index = msg->ranges.size() - 1;
    bool to_right = min_range_index < straight_index;
    unsigned long delta_from_perpendicular_right = min_range_index;
    unsigned long delta_from_perpendicular_left = leftmost_index - min_range_index;
    return LaserAnalysis(min_range_index, min_range, in_sight, near, leftmost_index, straight_index, to_right,
                         delta_from_perpendicular_right, delta_from_perpendicular_left);
}
