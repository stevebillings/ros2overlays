//
// Created by stevebillings on 11/21/22.
//

#ifndef OBSTACLE_HUGGER_VELOCITY_H
#define OBSTACLE_HUGGER_VELOCITY_H


class Velocity {
public:
    Velocity(double forward, double yaw);
    double get_forward();
    double get_yaw();
private:
    double forward_;
    double yaw_;
};


#endif //OBSTACLE_HUGGER_VELOCITY_H
