#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node {
	public:
		SubscriberNode() : Node("subscriber_node") {
			subscriber_ = create_subscription<std_msgs::msg::Int32>("sb_int_topic", 10,
					std::bind(&SubscriberNode::callback, this, _1));
		}

		void callback(const std_msgs::msg::Int32::SharedPtr msg) {
			RCLCPP_INFO(get_logger(), "Received %d from topic sb_int_topic", msg->data);
		}

	private:
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
};
