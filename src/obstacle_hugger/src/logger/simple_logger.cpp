//
// Created by stevebillings on 12/12/22.
//

#include <rclcpp/logging.hpp>
#include "simple_logger.h"

void SimpleLogger::log(const char* msg) const {
  RCLCPP_INFO(logger_, msg);
}
