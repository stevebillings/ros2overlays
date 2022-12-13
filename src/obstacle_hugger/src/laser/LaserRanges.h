//
// Created by stevebillings on 12/12/22.
//

#ifndef OBSTACLE_HUGGER_LASERRANGES_H
#define OBSTACLE_HUGGER_LASERRANGES_H


#include <vector>

class LaserRanges
{
public:
  LaserRanges(std::vector<float> ranges) : ranges_(ranges){};
  std::vector<float> get_ranges() const;
private:
  std::vector<float> ranges_;
};


#endif //OBSTACLE_HUGGER_LASERRANGES_H
