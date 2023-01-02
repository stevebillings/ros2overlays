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

std::string LaserAnalysis::toString() const
{
  nearestSighting_.getRangeIndex();
  nearestSighting_.getRange();
  std::string description = "";

  description.append("nearest range index: ");
  description.append(std::to_string(nearestSighting_.getRangeIndex()));

  description.append("; nearest range: ");
  description.append(std::to_string(nearestSighting_.getRange()));

  description.append("; in sight?: ");
  description.append(std::to_string(in_sight_));

  description.append("; near?: ");
  description.append(std::to_string(near_));

  description.append("; too near?: ");
  description.append(std::to_string(too_near_));

  description.append("; to right?: ");
  description.append(std::to_string(to_right_));

  description.append("; delta from perpendicular: ");
  description.append(std::to_string(delta_from_perpendicular_));

  return description;
}
