
#ifndef OBSTACLE_HUGGER_VELOCITY_CALCULATOR_H
#define OBSTACLE_HUGGER_VELOCITY_CALCULATOR_H

#include "../laser/laser_characteristics.h"
#include "../laser/laser_analysis.h"
#include "velocity.h"

class VelocityCalculator
{
public:
  VelocityCalculator();
  Velocity toApproach(const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const;
  Velocity toParallel(const LaserAnalysis& laser_analysis) const;

private:
  double PARALLEL_X_VELOCITY = 1.5;
  double DELTA_TO_YAW_MULTIPLIER = 0.01;
};

#endif  // OBSTACLE_HUGGER_VELOCITY_CALCULATOR_H
