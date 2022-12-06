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
  unsigned long get_leftmost_index() const;
  unsigned long get_straight_index() const;

private:
  unsigned long leftmost_index_;
  unsigned long straight_index_;
};

#endif  // OBSTACLE_HUGGER_LASER_CHARACTERISTICS_H