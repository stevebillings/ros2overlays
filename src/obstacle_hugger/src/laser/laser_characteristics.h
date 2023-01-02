//
// Created by stevebillings on 12/4/22.
//

#ifndef OBSTACLE_HUGGER_LASER_CHARACTERISTICS_H
#define OBSTACLE_HUGGER_LASER_CHARACTERISTICS_H

class LaserCharacteristics
{
public:
  LaserCharacteristics(const unsigned long leftmost_index, const unsigned long straight_index)
    : leftmost_index_(leftmost_index), straight_index_(straight_index){};
  LaserCharacteristics(const LaserCharacteristics& other)
    : leftmost_index_(other.leftmost_index_), straight_index_(other.straight_index_){};
  unsigned long getLeftmostIndex() const;
  unsigned long getStraightIndex() const;

private:
  unsigned long leftmost_index_;
  unsigned long straight_index_;
};

#endif  // OBSTACLE_HUGGER_LASER_CHARACTERISTICS_H
