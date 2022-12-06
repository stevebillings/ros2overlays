//
// Created by stevebillings on 12/4/22.
//

#include "laser_characteristics.h"

unsigned long LaserCharacteristics::get_leftmost_index() const
{
  return leftmost_index_;
}

unsigned long LaserCharacteristics::get_straight_index() const
{
  return straight_index_;
}