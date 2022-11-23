
#include "VelocityCalculator.h"

VelocityCalculator::VelocityCalculator() {}

Velocity VelocityCalculator::toApproach(LaserAnalysis laserAnalysis) {
    double yaw_delta_factor = 0.01;
    if (laserAnalysis.is_near()) {
        return Velocity(0.0, 0.0);
    }
    double x = laserAnalysis.get_min_range() / 2.0;
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
//    if (laserAnalysis.is_too_near()) {
//        printf("Obstacle is too near; stopping\n");
//        return Velocity(0.0, 0.0);
//    }
    printf("Calculating velocities to parallel obstacle\n");
    double x = 1.0;
    double yaw;
    if (laserAnalysis.is_to_right()) {
        printf("Obstacle is to the right\n");
        yaw = laserAnalysis.get_delta_from_perpendicular_right() * yaw_delta_factor;
    } else {
        printf("Obstacle is to the left\n");
        yaw = (laserAnalysis.get_delta_from_perpendicular_left()) * yaw_delta_factor * -1;
    }
    printf("Calculated velocities: x: %lf; yaw: %lf\n", x, yaw);
    return Velocity(x, yaw);
}