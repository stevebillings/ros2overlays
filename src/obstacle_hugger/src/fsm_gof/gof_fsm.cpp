
#include <iostream>
#include "states.h"

class Sandbox {
public:
  void play() const {
    States states;
    State* cur_state = states.get_state(FsmState::SEARCH);
    std::cout << "states.get_state() returned: " << cur_state->name() << "\n";
  }
};

int main(int argc, char* argv[])
{
  Sandbox sandbox;
  sandbox.play();
//  rclcpp::init(argc, argv);
//  auto node = std::make_shared<ObstacleHuggingNode>();
//  rclcpp::spin(node);
//  rclcpp::shutdown();
//  return 0;
}
