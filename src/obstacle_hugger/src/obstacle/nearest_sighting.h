//
// Created by stevebillings on 12/4/22.
//

#ifndef OBSTACLE_HUGGER_NEAREST_SIGHTING_H
#define OBSTACLE_HUGGER_NEAREST_SIGHTING_H

class NearestSighting
{
public:
  NearestSighting(const unsigned long range_index, double range) : range_index_(range_index), range_(range){};
  NearestSighting(const NearestSighting& src) : range_index_(src.getRangeIndex()), range_(src.getRange()){};
  unsigned long getRangeIndex() const;
  double getRange() const;

private:
  const unsigned long range_index_;
  const double range_;
};

#endif  // OBSTACLE_HUGGER_NEAREST_SIGHTING_H
