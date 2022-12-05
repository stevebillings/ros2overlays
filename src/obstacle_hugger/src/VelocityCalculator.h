
#ifndef OBSTACLE_HUGGER_VELOCITYCALCULATOR_H
#define OBSTACLE_HUGGER_VELOCITYCALCULATOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "LaserCharacteristics.h"
#include "LaserAnalysis.h"
#include "Velocity.h"

class VelocityCalculator {
public:
    VelocityCalculator();
    Velocity toApproach(const rclcpp::Logger& logger, const LaserCharacteristics& laserCharacteristics, const LaserAnalysis& laserAnalysis);
    Velocity toParallel(const rclcpp::Logger& logger, const LaserAnalysis& laserAnalysis);
};

#endif //OBSTACLE_HUGGER_VELOCITYCALCULATOR_H
