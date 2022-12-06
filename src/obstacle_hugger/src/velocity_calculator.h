
#ifndef OBSTACLE_HUGGER_VELOCITY_CALCULATOR_H
#define OBSTACLE_HUGGER_VELOCITY_CALCULATOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "laser_characteristics.h"
#include "laser_analysis.h"
#include "velocity.h"

class VelocityCalculator
{
public:
  VelocityCalculator();
  Velocity toApproach(const rclcpp::Logger& logger, const LaserCharacteristics& laser_characteristics,
                      const LaserAnalysis& laser_analysis);
  Velocity toParallel(const rclcpp::Logger& logger, const LaserAnalysis& laser_analysis);
};

#endif  // OBSTACLE_HUGGER_VELOCITY_CALCULATOR_H
