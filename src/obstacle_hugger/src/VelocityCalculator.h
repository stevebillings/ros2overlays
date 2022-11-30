
#ifndef OBSTACLE_HUGGER_VELOCITYCALCULATOR_H
#define OBSTACLE_HUGGER_VELOCITYCALCULATOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "LaserAnalysis.h"
#include "Velocity.h"

class VelocityCalculator {
public:
    VelocityCalculator();
    Velocity toApproach(rclcpp::Logger& logger, LaserAnalysis laserAnalysis);
    Velocity toParallel(rclcpp::Logger& logger, LaserAnalysis laserAnalysis);
};

#endif //OBSTACLE_HUGGER_VELOCITYCALCULATOR_H
