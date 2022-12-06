
#ifndef OBSTACLE_HUGGER_LASER_ANALYZER_H
#define OBSTACLE_HUGGER_LASER_ANALYZER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_analysis.h"
#include "laser_characteristics.h"

class LaserAnalyzer
{
public:
  LaserCharacteristics determine_characteristics(sensor_msgs::msg::LaserScan::SharedPtr msg) const;
  LaserAnalysis analyze(const rclcpp::Logger& logger, const LaserCharacteristics& laserCharacteristics,
                        sensor_msgs::msg::LaserScan::SharedPtr msg) const;

private:
  constexpr static double DIST_WITHIN_SIGHT = 9.5;
  constexpr static double DIST_NEAR = 4.0;
  constexpr static double DIST_TOO_NEAR = 1.5;
};

#endif  // OBSTACLE_HUGGER_LASER_ANALYZER_H
