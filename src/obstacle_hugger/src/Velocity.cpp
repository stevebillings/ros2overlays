//
// Created by stevebillings on 11/21/22.
//

#include "Velocity.h"

Velocity::Velocity(double forward, double yaw) {
    forward_ = forward;
    yaw_ = yaw;
}

double Velocity::get_forward() {
    return forward_;
}

double Velocity::get_yaw() {
    return yaw_;
}