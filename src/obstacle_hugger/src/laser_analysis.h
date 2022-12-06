
#ifndef OBSTACLE_HUGGER_LASER_ANALYSIS_H
#define OBSTACLE_HUGGER_LASER_ANALYSIS_H

#include "nearest_sighting.h"

class LaserAnalysis
{
public:
  LaserAnalysis(const NearestSighting& nearestSighting, const bool in_sight, const bool near, const bool too_near,
                const bool to_right, const unsigned long delta_from_perpendicular_right,
                const unsigned long delta_from_perpendicular_left)
    : nearestSighting_(nearestSighting)
    , in_sight_(in_sight)
    , near_(near)
    , too_near_(too_near)
    , to_right_(to_right)
    , delta_from_perpendicular_right_(delta_from_perpendicular_right)
    , delta_from_perpendicular_left_(delta_from_perpendicular_left){};
  LaserAnalysis(const LaserAnalysis& src)
    : nearestSighting_(src.getNearestSighting())
    , in_sight_(src.isInSight())
    , near_(src.isNear())
    , too_near_(src.isTooNear())
    , to_right_(src.isToRight())
    , delta_from_perpendicular_right_(src.getDeltaFromPerpendicularRight())
    , delta_from_perpendicular_left_(src.getDeltaFromPerpendicularLeft()){};
  const NearestSighting getNearestSighting() const;
  bool isInSight() const;
  bool isNear() const;
  bool isTooNear() const;
  bool isToRight() const;
  // TODO almost certainly don't need both of these:
  unsigned long getDeltaFromPerpendicularRight() const;
  unsigned long getDeltaFromPerpendicularLeft() const;

private:
  const NearestSighting nearestSighting_;
  const bool in_sight_;
  const bool near_;
  const bool too_near_;
  const bool to_right_;
  const unsigned long delta_from_perpendicular_right_;
  const unsigned long delta_from_perpendicular_left_;
};

#endif  // OBSTACLE_HUGGER_LASER_ANALYSIS_H
