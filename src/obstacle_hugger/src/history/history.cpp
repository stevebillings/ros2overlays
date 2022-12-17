#include "history.h"

void History::set_time_lost(double time_lost)
{
  time_lost_ = time_lost;
}
double History::get_time_lost() const {
  return time_lost_;
}

void History::set_obstacle_last_seen_time(double obstacle_last_seen_time, bool seen_to_right)
{
  obstacle_last_seen_time_ = obstacle_last_seen_time;
  seen_to_right_ = seen_to_right;
}

double History::get_obstacle_last_seen_time() const {
  return obstacle_last_seen_time_;
}

bool History::was_seen_to_right() const {
  return seen_to_right_;
}

bool History::has_obstacle_ever_been_seen() const
{
  return obstacle_last_seen_time_ != 0.0l;
}