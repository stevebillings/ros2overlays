#include "LaserAnalyzer.h"

LaserAnalyzer::LaserAnalyzer() {}

LaserAnalysis LaserAnalyzer::analyze(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    unsigned long index_dead_ahead = msg->ranges.size() / 2;
    int cur_range_index = 0;
    double min_range = 1000.0;
    unsigned long min_range_index = index_dead_ahead;
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
    return LaserAnalysis(min_range_index, min_range, in_sight, near, leftmost_index);
}
