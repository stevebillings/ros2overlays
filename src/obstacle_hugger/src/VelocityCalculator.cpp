
#include "VelocityCalculator.h"

VelocityCalculator::VelocityCalculator() {}

Velocity VelocityCalculator::toApproach(rclcpp::Logger& logger, LaserAnalysis laserAnalysis) {
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
    RCLCPP_INFO(logger, "toApproach() calculated: x: %lf, yaw: %lf", x, yaw);
    return Velocity(x, yaw);
}
Velocity VelocityCalculator::toParallel(rclcpp::Logger& logger, LaserAnalysis laserAnalysis) {
    double yaw_delta_factor = 0.01;
    RCLCPP_INFO(logger, "Calculating velocities to parallel obstacle\n");
    double x = 1.0;
    double yaw;
    if (laserAnalysis.is_in_sight()) {
        if (laserAnalysis.is_to_right()) {
            RCLCPP_INFO(logger, "Obstacle is to the right\n");
            yaw = laserAnalysis.get_delta_from_perpendicular_right() * yaw_delta_factor;
        } else {
            RCLCPP_INFO(logger, "Obstacle is to the left\n");
            yaw = (laserAnalysis.get_delta_from_perpendicular_left()) * yaw_delta_factor * -1;
        }
    }
    RCLCPP_INFO(logger, "Calculated velocities: x: %lf; yaw: %lf\n", x, yaw);
    return Velocity(x, yaw);
}