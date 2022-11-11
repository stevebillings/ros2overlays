#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

// I think I'm abandoning this one; not sure it's (range value-less) output is going to be useful
// If resurrected, see subscriber.cpp for the right way to calculate angle indices for left/straight/right
class SubscriberNode : public rclcpp::Node {
	public:
		SubscriberNode() : Node("laser_monitor_node") {
            laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", 10,
                                                                                      std::bind(&SubscriberNode::callback, this, _1));
            publisher_ = this->create_publisher<std_msgs::msg::String>("obstacle_position", 10);
		}

		void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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
            auto obstacle_position_message = std_msgs::msg::String();
            if (min_range < 2.0) {
                if (min_range_index < 350) {
                    RCLCPP_INFO(get_logger(), "Obstacle on right: %f meters", min_range);
                    obstacle_position_message.data = "right";
                } else if (min_range_index > 370) {
                    RCLCPP_INFO(get_logger(), "Obstacle on left: %f meters", min_range);
                    obstacle_position_message.data = "left";
                } else {
                    RCLCPP_INFO(get_logger(), "Obstacle straight ahead: %f meters", min_range);
                    obstacle_position_message.data = "ahead";
                }
            } else {
                RCLCPP_INFO(get_logger(), "Clear");
                obstacle_position_message.data = "";
            }
            publisher_->publish(obstacle_position_message);
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
