#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node_pub = std::make_shared<PublisherNode>();
  auto node_sub = std::make_shared<SubscriberNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);

  // rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node_pub);
  executor.add_node(node_sub);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
