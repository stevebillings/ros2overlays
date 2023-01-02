
#include "velocity_calculator.h"

VelocityCalculator::VelocityCalculator()
{
}

Velocity VelocityCalculator::toApproach(const LaserCharacteristics& laser_characteristics,
                                        const LaserAnalysis& laser_analysis) const
{
  if (laser_analysis.isNear() || laser_analysis.isTooNear())
  {
    return Velocity(0.0, 0.0);
  }
  double x = laser_analysis.getNearestSighting().getRange() / 4.0;
  double yaw;
  if (laser_analysis.isToRight())
  {
    long index_offset = laser_characteristics.getStraightIndex() - laser_analysis.getNearestSighting().getRangeIndex();
    yaw = (double)index_offset * DELTA_TO_YAW_MULTIPLIER * -1.0;
  }
  else
  {
    long index_offset = laser_analysis.getNearestSighting().getRangeIndex() - laser_characteristics.getStraightIndex();
    yaw = (double)index_offset * DELTA_TO_YAW_MULTIPLIER;
  }
  return Velocity(x, yaw);
}
Velocity VelocityCalculator::toParallel(const LaserAnalysis& laser_analysis) const
{
  double yaw;
  if (laser_analysis.isInSight())
  {
    if (laser_analysis.isToRight())
    {
      yaw = laser_analysis.getDeltaFromPerpendicular() * DELTA_TO_YAW_MULTIPLIER;
    }
    else
    {
      yaw = (laser_analysis.getDeltaFromPerpendicular()) * DELTA_TO_YAW_MULTIPLIER * -1;
    }
  }
  return Velocity(PARALLEL_X_VELOCITY, yaw);
}