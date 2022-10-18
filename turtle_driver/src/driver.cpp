#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/teleport_relative.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("turtle_driver_node");
	rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr client =
		node->create_client<turtlesim::srv::TeleportRelative>("/turtle1/teleport_relative");

	while (!client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service unavailable, waiting again...");
	}
	auto request = std::make_shared<turtlesim::srv::TeleportRelative::Request>();
	request->linear = 1.0;
	request->angular = 1.0;
	auto result = client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request succeeded!");
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Request failed");
	}
	rclcpp::shutdown();
	return 0;
}
