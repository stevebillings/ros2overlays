//
// Created by stevebillings on 11/21/22.
//

#include "velocity.h"

Velocity::Velocity(double forward, double yaw)
{
  forward_ = forward;
  yaw_ = yaw;
}

Velocity Velocity::createStopped()
{
  return Velocity(0.0, 0.0);
}

Velocity Velocity::createSearchSpinRight()
{
  return Velocity(0.0, -0.5);
}

Velocity Velocity::createSearchSpinLeft()
{
  return Velocity(0.0, 0.5);
}

Velocity Velocity::createReverse()
{
  return Velocity(-1.0, 0.0);
}

double Velocity::getForward() const
{
  return forward_;
}

double Velocity::getYaw() const
{
  return yaw_;
}