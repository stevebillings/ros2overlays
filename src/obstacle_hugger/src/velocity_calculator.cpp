
#include "velocity_calculator.h"

VelocityCalculator::VelocityCalculator()
{
}

Velocity VelocityCalculator::toApproach(const rclcpp::Logger& logger, const LaserCharacteristics& laserCharacteristics,
                                        const LaserAnalysis& laserAnalysis)
{
  double yaw_delta_factor = 0.01;
  if (laserAnalysis.is_near())
  {
    return Velocity(0.0, 0.0);
  }
  double x = laserAnalysis.get_nearest_sighting().get_range() / 4.0;
  double yaw;
  if (laserAnalysis.is_to_right())
  {
    long index_offset =
        laserCharacteristics.get_straight_index() - laserAnalysis.get_nearest_sighting().get_range_index();
    yaw = (double)index_offset * yaw_delta_factor * -1.0;
  }
  else
  {
    long index_offset =
        laserAnalysis.get_nearest_sighting().get_range_index() - laserCharacteristics.get_straight_index();
    yaw = (double)index_offset * yaw_delta_factor;
  }
  RCLCPP_INFO(logger, "toApproach() calculated: x: %lf, yaw: %lf", x, yaw);
  return Velocity(x, yaw);
}
Velocity VelocityCalculator::toParallel(const rclcpp::Logger& logger, const LaserAnalysis& laserAnalysis)
{
  double yaw_delta_factor = 0.01;
  RCLCPP_INFO(logger, "Calculating velocities to parallel obstacle\n");
  double x = 1.5;
  double yaw;
  if (laserAnalysis.is_in_sight())
  {
    if (laserAnalysis.is_to_right())
    {
      RCLCPP_INFO(logger, "Obstacle is to the right\n");
      yaw = laserAnalysis.get_delta_from_perpendicular_right() * yaw_delta_factor;
    }
    else
    {
      RCLCPP_INFO(logger, "Obstacle is to the left\n");
      yaw = (laserAnalysis.get_delta_from_perpendicular_left()) * yaw_delta_factor * -1;
    }
  }
  RCLCPP_INFO(logger, "Calculated velocities: x: %lf; yaw: %lf\n", x, yaw);
  return Velocity(x, yaw);
}