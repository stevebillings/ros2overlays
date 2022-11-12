#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node {
	public:
		DriverNode() : Node("driver_node") {
			publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			timer_ = create_wall_timer(1s, std::bind(&DriverNode::timer_callback, this));
		}

		void timer_callback() {
			if (state_ == 0) {
                message_.linear.x = 1.0;
                message_.angular.z = 0.0;
                state_ = 1;
			} else if (state_ == 1) {
                message_.linear.x = 0.0;
                message_.angular.z = 2.0;
                state_ = 2;
            } else {
                message_.linear.x = 0.0;
                message_.angular.z = 0.0;
                state_ = 0;
            }
			publisher_->publish(message_);
		}
	private:
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        geometry_msgs::msg::Twist message_;
		rclcpp::TimerBase::SharedPtr timer_;
		int state_ = 0;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DriverNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
