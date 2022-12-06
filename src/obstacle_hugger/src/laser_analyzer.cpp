#include "laser_analyzer.h"

LaserCharacteristics LaserAnalyzer::determineCharacteristics(sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
  unsigned long straight_index = msg->ranges.size() / 2;
  unsigned long leftmost_index = msg->ranges.size() - 1;
  return LaserCharacteristics(leftmost_index, straight_index);
}

LaserAnalysis LaserAnalyzer::analyze(const rclcpp::Logger& logger, const LaserCharacteristics& laserCharacteristics,
                                     sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
  int cur_range_index = 0;
  double min_range = 1000.0;
  unsigned long min_range_index = laserCharacteristics.getStraightIndex();
  for (auto this_range : msg->ranges)
  {
    if (this_range < min_range)
    {
      min_range = this_range;
      min_range_index = cur_range_index;
    }
    cur_range_index++;
  }
  RCLCPP_INFO(logger, "min_range_index: %ld, min_range: %lf", min_range_index, min_range);
  bool in_sight = min_range < DIST_WITHIN_SIGHT;
  bool near = min_range < DIST_NEAR;
  bool too_near = min_range < DIST_TOO_NEAR;

  bool to_right = min_range_index < laserCharacteristics.getStraightIndex();
  unsigned long delta_from_perpendicular_right = min_range_index;
  unsigned long delta_from_perpendicular_left = laserCharacteristics.getLeftmostIndex() - min_range_index;

  // TODO how does this memory get freed?
  NearestSighting nearestSighting = NearestSighting(min_range_index, min_range);
  return LaserAnalysis(nearestSighting, in_sight, near, too_near, to_right, delta_from_perpendicular_right,
                       delta_from_perpendicular_left);
}