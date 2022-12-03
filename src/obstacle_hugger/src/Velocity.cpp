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
    return Velocity(0.0, -0.5);
}

Velocity Velocity::createSearchSpinLeft() {
    return Velocity(0.0, 0.5);
}

Velocity Velocity::createReverse() {
    return Velocity(-1.0, 0.0);
}

double Velocity::get_forward() const {
    return forward_;
}

double Velocity::get_yaw() const {
    return yaw_;
}