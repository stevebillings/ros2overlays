
#include "velocity_calculator.h"

VelocityCalculator::VelocityCalculator()
{
}

Velocity VelocityCalculator::toApproach(const rclcpp::Logger& logger, const LaserCharacteristics& laser_characteristics,
                                        const LaserAnalysis& laser_analysis)
{
  double yaw_delta_factor = 0.01;
  if (laser_analysis.isNear())
  {
    return Velocity(0.0, 0.0);
  }
  double x = laser_analysis.getNearestSighting().getRange() / 4.0;
  double yaw;
  if (laser_analysis.isToRight())
  {
    long index_offset =
            laser_characteristics.getStraightIndex() - laser_analysis.getNearestSighting().getRangeIndex();
    yaw = (double)index_offset * yaw_delta_factor * -1.0;
  }
  else
  {
    long index_offset =
            laser_analysis.getNearestSighting().getRangeIndex() - laser_characteristics.getStraightIndex();
    yaw = (double)index_offset * yaw_delta_factor;
  }
  RCLCPP_INFO(logger, "toApproach() calculated: x: %lf, yaw: %lf", x, yaw);
  return Velocity(x, yaw);
}
Velocity VelocityCalculator::toParallel(const rclcpp::Logger& logger, const LaserAnalysis& laser_analysis)
{
  double yaw_delta_factor = 0.01;
  RCLCPP_INFO(logger, "Calculating velocities to parallel obstacle\n");
  double x = 1.5;
  double yaw;
  if (laser_analysis.isInSight())
  {
    if (laser_analysis.isToRight())
    {
      RCLCPP_INFO(logger, "Obstacle is to the right\n");
      yaw = laser_analysis.getDeltaFromPerpendicularRight() * yaw_delta_factor;
    }
    else
    {
      RCLCPP_INFO(logger, "Obstacle is to the left\n");
      yaw = (laser_analysis.getDeltaFromPerpendicularLeft()) * yaw_delta_factor * -1;
    }
  }
  RCLCPP_INFO(logger, "Calculated velocities: x: %lf; yaw: %lf\n", x, yaw);
  return Velocity(x, yaw);
}