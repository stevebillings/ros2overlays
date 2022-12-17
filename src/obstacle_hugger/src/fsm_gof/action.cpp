
#include "action.h"

Velocity Action::get_velocity() const
{
  return velocity_;
}
FsmState Action::get_state() const
{
  return state_;
}