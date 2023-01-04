#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Vff {
public:
  [[nodiscard]] std::vector<float> getVff(double laser_angle_min, double laser_angle_increment, std::vector<float>  const & laser_ranges) const;
  int getInt(std::vector<int>  & stuff);
};