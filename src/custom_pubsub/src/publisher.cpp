#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node {
	public:
		PublisherNode() : Node("publisher_node"), count_(0) {
			publisher_ = create_publisher<tutorial_interfaces::msg::Num>("sb_int_topic", 10);
			timer_ = create_wall_timer(1s, std::bind(&PublisherNode::timer_callback, this));
		}

	private:
		void timer_callback() {
			auto message = tutorial_interfaces::msg::Num();
			message.num = this->count_++;
			RCLCPP_INFO(get_logger(), "Publishing: %ld", message.num);
			publisher_->publish(message);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
		size_t count_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PublisherNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
