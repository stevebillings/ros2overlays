//
// Created by stevebillings on 11/21/22.
//

#include "Velocity.h"

Velocity::Velocity(double forward, double yaw) {
    forward_ = forward;
    yaw_ = yaw;
}

Velocity Velocity::createStopped() {
    return Velocity(0.0, 0.0);
}

Velocity Velocity::createSearchSpinRight() {
    return Velocity(0.0, -0.1);
}

double Velocity::get_forward() {
    return forward_;
}

double Velocity::get_yaw() {
    return yaw_;
}