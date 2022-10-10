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
			RCLCPP_INFO(get_logger(), "Hello %d", msg->data);
		}

	private:
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
