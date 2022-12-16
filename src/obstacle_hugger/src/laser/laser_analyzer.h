
#ifndef OBSTACLE_HUGGER_LASER_ANALYZER_H
#define OBSTACLE_HUGGER_LASER_ANALYZER_H

#include "laser_analysis.h"
#include "laser_characteristics.h"

class LaserAnalyzer
{
public:
  LaserCharacteristics determineCharacteristics(const std::vector<float>& laser_ranges) const;
  LaserAnalysis analyze(const LaserCharacteristics& laserCharacteristics, const std::vector<float>& laser_ranges) const;

private:
  constexpr static double DIST_WITHIN_SIGHT = 9.5;
  constexpr static double DIST_NEAR = 4.0;
  constexpr static double DIST_TOO_NEAR = 1.5;
};

#endif  // OBSTACLE_HUGGER_LASER_ANALYZER_H
