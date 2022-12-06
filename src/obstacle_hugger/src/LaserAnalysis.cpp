//
// Created by stevebillings on 11/21/22.
//

#include "LaserAnalysis.h"

const NearestSighting LaserAnalysis::get_nearest_sighting() const
{
  return nearestSighting_;
}

bool LaserAnalysis::is_in_sight() const
{
  return in_sight_;
}

bool LaserAnalysis::is_near() const
{
  return near_;
}

bool LaserAnalysis::is_too_near() const
{
  return too_near_;
}

bool LaserAnalysis::is_to_right() const
{
  return to_right_;
}

unsigned long LaserAnalysis::get_delta_from_perpendicular_right() const
{
  return delta_from_perpendicular_right_;
}
unsigned long LaserAnalysis::get_delta_from_perpendicular_left() const
{
  return delta_from_perpendicular_left_;
}
