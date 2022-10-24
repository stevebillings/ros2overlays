#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node {
	public:
		SubscriberNode() : Node("subscriber_node") {
			subscription_ = create_subscription<tutorial_interfaces::msg::Num>("sb_int_topic", 10,
					std::bind(&SubscriberNode::callback, this, _1));
		}

	private:
		void callback(const tutorial_interfaces::msg::Num::SharedPtr msg) {
			RCLCPP_INFO(get_logger(), "Hello %ld", msg->num);
		}

		rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
