
#ifndef OBSTACLE_HUGGER_LASERANALYZER_H
#define OBSTACLE_HUGGER_LASERANALYZER_H

#include "sensor_msgs/msg/laser_scan.hpp"
#include "LaserAnalysis.h"

class LaserAnalyzer {
public:
    LaserAnalyzer();
    LaserAnalysis analyze(sensor_msgs::msg::LaserScan::SharedPtr msg);
private:
    constexpr static double DIST_WITHIN_SIGHT = 8.0;
    constexpr static double DIST_NEAR = 3.0;
};

#endif //OBSTACLE_HUGGER_LASERANALYZER_H
