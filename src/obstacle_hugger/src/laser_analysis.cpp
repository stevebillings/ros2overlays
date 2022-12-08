//
// Created by stevebillings on 11/21/22.
//

#include "laser_analysis.h"

const NearestSighting LaserAnalysis::getNearestSighting() const
{
  return nearestSighting_;
}

bool LaserAnalysis::isInSight() const
{
  return in_sight_;
}

bool LaserAnalysis::isNear() const
{
  return near_;
}

bool LaserAnalysis::isTooNear() const
{
  return too_near_;
}

bool LaserAnalysis::isToRight() const
{
  return to_right_;
}

unsigned long LaserAnalysis::getDeltaFromPerpendicular() const
{
  return delta_from_perpendicular_;
}
