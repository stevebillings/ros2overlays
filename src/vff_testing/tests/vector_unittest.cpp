#include "vff_testing/vector.hpp"
#include <gtest/gtest.h>
#include <math.h>

TEST(VECTOR_UNITTEST, AHEAD) {
  auto v = Vector(1.0, 0.0);

  ASSERT_NEAR(v.getAngle(), 0.0, 0.001);
  ASSERT_NEAR(v.getMagnitude(), 1.0, 0.001);
}

TEST(VECTOR_UNITTEST, LEFT) {
  auto v = Vector(0.0, 1.0);

  ASSERT_NEAR(v.getAngle(), M_PI / 2.0, 0.01);
  ASSERT_NEAR(v.getMagnitude(), 1.0, 0.001);
}

TEST(VECTOR_UNITTEST, RIGHT_45) {
  auto v = Vector(1.0, -1.0);

  ASSERT_NEAR(v.getAngle(), -1 * M_PI / 4.0, 0.01);
  ASSERT_NEAR(v.getMagnitude(), sqrt(2.0), 0.001);
}