//
// Created by stevebillings on 12/4/22.
//

#include "laser_characteristics.h"

unsigned long LaserCharacteristics::getLeftmostIndex() const
{
  return leftmost_index_;
}

unsigned long LaserCharacteristics::getStraightIndex() const
{
  return straight_index_;
}