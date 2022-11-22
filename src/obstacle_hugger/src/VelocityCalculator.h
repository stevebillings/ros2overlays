
#ifndef OBSTACLE_HUGGER_VELOCITYCALCULATOR_H
#define OBSTACLE_HUGGER_VELOCITYCALCULATOR_H

#include "rclcpp/rclcpp.hpp"
#include "LaserAnalysis.h"
#include "Velocity.h"

class VelocityCalculator {
public:
    VelocityCalculator();
    Velocity toApproach(LaserAnalysis laserAnalysis);
    Velocity toParallel(LaserAnalysis laserAnalysis);
};

#endif //OBSTACLE_HUGGER_VELOCITYCALCULATOR_H
