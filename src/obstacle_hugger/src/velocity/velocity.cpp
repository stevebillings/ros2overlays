//
// Created by stevebillings on 11/21/22.
//

#include "velocity.h"

Velocity::Velocity(double forward, double yaw)
{
  forward_ = forward;
  yaw_ = yaw;
}

Velocity Velocity::create_stopped()
{
  return Velocity(0.0, 0.0);
}

Velocity Velocity::create_spin_right()
{
  return Velocity(0.0, -0.5);
}

Velocity Velocity::create_spin_left()
{
  return Velocity(0.0, 0.5);
}

Velocity Velocity::create_reverse()
{
  return Velocity(-1.0, 0.0);
}

double Velocity::get_forward() const
{
  return forward_;
}

double Velocity::get_yaw() const
{
  return yaw_;
}