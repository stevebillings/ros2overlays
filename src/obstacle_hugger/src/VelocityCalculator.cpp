
#include "VelocityCalculator.h"

VelocityCalculator::VelocityCalculator() {}

Velocity VelocityCalculator::toApproach(LaserAnalysis laserAnalysis) {
    double yaw_delta_factor = 0.01;
    if (laserAnalysis.is_near()) {
        return Velocity(0.0, 0.0);
    }
    double x = laserAnalysis.get_min_range();
    double yaw;
    if (laserAnalysis.is_to_right()) {
        yaw = (laserAnalysis.get_straight_index() - laserAnalysis.get_min_range_index()) * yaw_delta_factor * -1;
    } else {
        yaw = (laserAnalysis.get_min_range_index() - laserAnalysis.get_straight_index()) * yaw_delta_factor;
    }
    return Velocity(x, yaw);
}
Velocity VelocityCalculator::toParallel(LaserAnalysis laserAnalysis) {
    double yaw_delta_factor = 0.01;
    if (laserAnalysis.is_near()) {
        return Velocity(0.0, 0.0);
    }
    double x = laserAnalysis.get_min_range();
    double yaw;
    if (laserAnalysis.is_to_right()) {
        yaw = laserAnalysis.get_delta_from_perpendicular_right() * yaw_delta_factor;
    } else {
        yaw = (laserAnalysis.get_delta_from_perpendicular_left()) * yaw_delta_factor * -1;
    }
    return Velocity(x, yaw);
}