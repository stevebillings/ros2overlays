
#include <iostream>
#include "states.h"

class Sandbox {
public:
  void play() const {
    History history;
    LaserCharacteristics laser_characteristics = LaserCharacteristics(4ul, 2ul);
    NearestSighting nearest_sighting = NearestSighting(1ul, 4.0l);
    LaserAnalysis laser_analysis = LaserAnalysis(nearest_sighting, true, false, false,true, 1ul);
    States states;
    State* cur_state = states.get_state(FsmState::SEARCH);
    std::cout << "states.get_state(search) returned: " << cur_state->name() << "\n";
    Action action = cur_state->act(history, laser_characteristics, laser_analysis);

    cur_state = states.get_state(action.get_state());
    std::cout << "states.get_state() returned: " << cur_state->name() << "\n";
    action = cur_state->act(history, laser_characteristics, laser_analysis);

    cur_state = states.get_state(action.get_state());
    std::cout << "states.get_state() returned: " << cur_state->name() << "\n";
    action = cur_state->act(history, laser_characteristics, laser_analysis);

    cur_state = states.get_state(action.get_state());
    std::cout << "states.get_state() returned: " << cur_state->name() << "\n";
    action = cur_state->act(history, laser_characteristics, laser_analysis);

    cur_state = states.get_state(action.get_state());
    std::cout << "states.get_state() returned: " << cur_state->name() << "\n";
    action = cur_state->act(history, laser_characteristics, laser_analysis);
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
