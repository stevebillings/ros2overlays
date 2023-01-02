#include <vector>
#include "laser_analyzer.h"

LaserCharacteristics LaserAnalyzer::determineCharacteristics(const std::vector<float>& laser_ranges) const
{
  unsigned long straight_index = laser_ranges.size() / 2;
  unsigned long leftmost_index = laser_ranges.size() - 1;
  return LaserCharacteristics(leftmost_index, straight_index);
}

LaserAnalysis LaserAnalyzer::analyze(const LaserCharacteristics& laserCharacteristics,
                                     const std::vector<float>& laser_ranges) const
{
  int cur_range_index = 0;
  double min_range = 1000.0;
  unsigned long min_range_index = laserCharacteristics.getStraightIndex();
  for (float this_range : laser_ranges)
  {
    if (this_range < min_range)
    {
      min_range = this_range;
      min_range_index = cur_range_index;
    }
    cur_range_index++;
  }
  bool in_sight = min_range < DIST_WITHIN_SIGHT;
  bool near = min_range < DIST_NEAR;
  bool too_near = min_range < DIST_TOO_NEAR;

  bool to_right = min_range_index < laserCharacteristics.getStraightIndex();
  unsigned long delta_from_perpendicular = 0L;
  if (to_right)
  {
    delta_from_perpendicular = min_range_index;
  }
  else
  {
    delta_from_perpendicular = laserCharacteristics.getLeftmostIndex() - min_range_index;
  }
  // TODO how does this memory get freed?
  NearestSighting nearestSighting = NearestSighting(min_range_index, min_range);
  return LaserAnalysis(nearestSighting, in_sight, near, too_near, to_right, delta_from_perpendicular);
}