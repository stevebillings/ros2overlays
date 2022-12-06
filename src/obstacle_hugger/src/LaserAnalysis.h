
#ifndef OBSTACLE_HUGGER_LASERANALYSIS_H
#define OBSTACLE_HUGGER_LASERANALYSIS_H

#include "NearestSighting.h"

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
    : nearestSighting_(src.get_nearest_sighting())
    , in_sight_(src.is_in_sight())
    , near_(src.is_near())
    , too_near_(src.is_too_near())
    , to_right_(src.is_to_right())
    , delta_from_perpendicular_right_(src.get_delta_from_perpendicular_right())
    , delta_from_perpendicular_left_(src.get_delta_from_perpendicular_left()){};
  const NearestSighting get_nearest_sighting() const;
  bool is_in_sight() const;
  bool is_near() const;
  bool is_too_near() const;
  bool is_to_right() const;
  unsigned long get_delta_from_perpendicular_right() const;
  unsigned long get_delta_from_perpendicular_left() const;

private:
  const NearestSighting nearestSighting_;
  const bool in_sight_;
  const bool near_;
  const bool too_near_;
  const bool to_right_;
  const unsigned long delta_from_perpendicular_right_;
  const unsigned long delta_from_perpendicular_left_;
};

#endif  // OBSTACLE_HUGGER_LASERANALYSIS_H
