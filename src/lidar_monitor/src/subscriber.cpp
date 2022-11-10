#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node {
	public:
		SubscriberNode() : Node("laser_monitor_node") {
			subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", 10,
					std::bind(&SubscriberNode::callback, this, _1));
		}

		void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
			/*
            RCLCPP_INFO(get_logger(), "Received LaserScan angle_min: %f; angle_max: %f; angle_increment: %f",
				msg->angle_min, msg->angle_max,
				msg->angle_increment);
			 */
			int i = 0;
			auto min_range = 999.999;
			int min_range_index = -1;
			for (auto this_range : msg->ranges) {
				if (this_range < min_range) {
					min_range = this_range;
					min_range_index = i;
				}
				i++;
			}
			// RCLCPP_INFO(get_logger(), "\tfound min range %f at index %d", min_range, min_range_index);
            int dead_ahead = msg->ranges.size() / 2;
            int ahead_width = msg->ranges.size() / 10;
            int ahead_width_from_center = ahead_width / 2;
            int right_limit = dead_ahead - ahead_width_from_center;
            int left_limit = dead_ahead + ahead_width_from_center;
            // RCLCPP_INFO(get_logger(), "Ahead is %d to %d", right_limit, left_limit);
            if (min_range < 2.0) {
                if (min_range_index < right_limit) {
                    RCLCPP_INFO(get_logger(), "Obstacle on right: %f meters", min_range);
                } else if (min_range_index > left_limit) {
                    RCLCPP_INFO(get_logger(), "Obstacle on left: %f meters", min_range);
                } else {
                    RCLCPP_INFO(get_logger(), "Obstacle straight ahead: %f meters", min_range);
                }
            } else {
                RCLCPP_INFO(get_logger(), "Clear");
            }
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
