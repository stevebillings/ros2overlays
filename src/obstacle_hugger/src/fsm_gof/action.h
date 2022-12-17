//
// Created by stevebillings on 12/17/22.
//

#ifndef OBSTACLE_HUGGER_ACTION_H
#define OBSTACLE_HUGGER_ACTION_H


#include "../common/fsm_state.h"
#include "../velocity/velocity.h"

class Action
{
public:
  Action(const Velocity& velocity, const FsmState state)
  : velocity_(velocity)
  , state_(state) {};
  Velocity get_velocity() const;
  FsmState get_state() const;
private:
  Velocity velocity_;
  FsmState state_;
};


#endif //OBSTACLE_HUGGER_ACTION_H
