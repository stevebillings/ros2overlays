#include <math.h>
#include "vff_testing/vector.hpp"

double Vector::getMagnitude() {
  return sqrt(endpoint_x_*endpoint_x_ + endpoint_y_*endpoint_y_);
}

double Vector::getAngle() {
  return atan2(endpoint_y_, endpoint_x_);
}
