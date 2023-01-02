//
// Created by stevebillings on 12/17/22.
//

#ifndef OBSTACLE_HUGGER_ACTION_H
#define OBSTACLE_HUGGER_ACTION_H

#include <optional>
#include "../common/fsm_state.h"
#include "../velocity/velocity.h"

class Action
{
public:
  Action(const Velocity& velocity, const FsmState state) : velocity_(velocity), state_(state){};
  Action(const FsmState state) : velocity_(std::nullopt), state_(state){};
  std::optional<Velocity> get_velocity() const;
  FsmState get_state() const;

private:
  std::optional<Velocity> velocity_;
  FsmState state_;
};

#endif  // OBSTACLE_HUGGER_ACTION_H
