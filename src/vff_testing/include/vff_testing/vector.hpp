//
// Created by stevebillings on 2/15/23.
//

#ifndef VFF_TESTING_VECTOR_H
#define VFF_TESTING_VECTOR_H

class Vector {
public:
  Vector(double endpoint_x, double endpoint_y) : endpoint_x_(endpoint_x), endpoint_y_(endpoint_y) {};
  double getAngle();
  double getMagnitude();
private:
  double endpoint_x_;
  double endpoint_y_;
};

#endif //VFF_TESTING_VECTOR_H
