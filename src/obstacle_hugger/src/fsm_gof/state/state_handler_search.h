
#ifndef OBSTACLE_HUGGER_STATE_HANDLER_SEARCH_H
#define OBSTACLE_HUGGER_STATE_HANDLER_SEARCH_H

#include "state_handler.h"

class StateHandlerSearch : public StateHandler
{
public:
  Action act(const History& history, const double current_time, const LaserCharacteristics& laser_characteristics,
             const LaserAnalysis& laser_analysis) const override;
  const char* name() const;

private:
  Action handleInSight(const LaserCharacteristics& laser_characteristics, const LaserAnalysis& laser_analysis) const;
  Action handleLostSight(const History& history) const;
  Action handleNeverSeen() const;
  Action handleRecentlyLost() const;
};

#endif  // OBSTACLE_HUGGER_STATE_HANDLER_SEARCH_H
