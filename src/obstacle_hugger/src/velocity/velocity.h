#ifndef OBSTACLE_HUGGER_VELOCITY_H
#define OBSTACLE_HUGGER_VELOCITY_H

class Velocity
{
public:
  Velocity(double forward, double yaw);
  static Velocity create_stopped();
  static Velocity create_spin_right();
  static Velocity create_spin_left();
  static Velocity create_reverse();
  double get_forward() const;
  double get_yaw() const;

private:
  double forward_;
  double yaw_;
};

#endif  // OBSTACLE_HUGGER_VELOCITY_H
