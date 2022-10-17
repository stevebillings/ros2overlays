#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node {
	public:
		PublisherNode() : Node("publisher_node") {
			publisher_ = create_publisher<std_msgs::msg::Int32>("sb_int_topic", 10);
			timer_ = create_wall_timer(1s, std::bind(&PublisherNode::timer_callback, this));
			RCLCPP_INFO(get_logger(), "Will be publishing ints to topic sb_int_topic");
		}

		void timer_callback() {
			message_.data ++;
			publisher_->publish(message_);
		}
	private:
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		std_msgs::msg::Int32 message_;
};
