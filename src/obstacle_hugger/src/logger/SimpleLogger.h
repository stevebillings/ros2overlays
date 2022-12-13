//
// Created by stevebillings on 12/12/22.
//

#ifndef OBSTACLE_HUGGER_SIMPLELOGGER_H
#define OBSTACLE_HUGGER_SIMPLELOGGER_H

#include <rclcpp/logger.hpp>

class SimpleLogger
{
public:
  SimpleLogger(const rclcpp::Logger& logger): logger_(logger){};
  void log(const char* msg) const;
private:
  const rclcpp::Logger& logger_;
};


#endif //OBSTACLE_HUGGER_SIMPLELOGGER_H
