
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "states.h"
#include "../laser/laser_analyzer.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class GofObstacleHuggingNode : public rclcpp::Node
{
public:
  GofObstacleHuggingNode() : Node("gof_obstacle_hugging_node")
  {
    laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "laser_scan", 10, std::bind(&GofObstacleHuggingNode::laserScanCallback, this, _1));
    timer_ = create_wall_timer(50ms, std::bind(&GofObstacleHuggingNode::controlCallback, this));
    drive_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    //setState(FsmState::SEARCH);
  }
private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_laser_scan_msg_ = std::move(msg);
  }

  void controlCallback()
  {
    RCLCPP_INFO(logger_, "======================");
    RCLCPP_INFO(logger_, "* FsmState: %s", cur_state_->name());
    if (last_laser_scan_msg_ == nullptr)
      return;  // ain't seen nothin' yet

    double current_time = now().seconds();
    init_laser_characteristics();
    LaserAnalysis laser_analysis = laser_analyzer_.analyze(*laser_characteristics_, last_laser_scan_msg_->ranges);
    update_history(laser_analysis);

    Action action = cur_state_->act(history_, current_time, *laser_characteristics_, laser_analysis);

    history_.set_time_entered_state(action.get_state(), current_time);
    cur_state_ = states_.get_state(action.get_state());
    std::optional<Velocity> new_velocity = action.get_velocity();
    if (new_velocity.has_value())
      set_velocity(new_velocity.value());
  }

  void update_history(const LaserAnalysis &laser_analysis)
  {
    if (laser_analysis.isInSight())
    {
      bool seenToRight = laser_analysis.isToRight();
      history_.set_obstacle_last_seen_time(now().seconds(), seenToRight);
    }
    double time_lost = calculate_time_lost(laser_analysis);
    history_.set_time_lost(time_lost);
  }

  void init_laser_characteristics()
  {
    if (laser_characteristics_ == nullptr)
    {
      LaserCharacteristics laser_characteristics = laser_analyzer_.determineCharacteristics(
          last_laser_scan_msg_->ranges);
      // Toss characteristics object onto heap so it sticks around for the life of the node
      laser_characteristics_ = new LaserCharacteristics(laser_characteristics);
    }
  }

  void set_velocity(const Velocity& velocity)
  {
    if ((abs(velocity.get_forward()) > 5.0) || (abs(velocity.get_yaw()) > 5.0))
    {
      RCLCPP_ERROR(logger_, "Invalid velocity: x: %lf, yaw: %lf", velocity.get_forward(), velocity.get_yaw());
      cur_state_ = states_.get_state(FsmState::ERROR);
      return;
    }
    geometry_msgs::msg::Twist drive_message;
    drive_message.linear.x = velocity.get_forward();
    drive_message.angular.z = velocity.get_yaw();
    drive_publisher_->publish(drive_message);
  }

  double calculate_time_lost(const LaserAnalysis &laser_analysis) const
  {
    double time_lost = 0.0;
    if (history_.has_obstacle_ever_been_seen() && !laser_analysis.isInSight())
    {
      time_lost = now().seconds() - history_.get_obstacle_last_seen_time();
    }
    return time_lost;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
  States states_ = States();
  State* cur_state_ = states_.get_state(FsmState::SEARCH);
  LaserCharacteristics* laser_characteristics_ = nullptr;
  LaserAnalyzer laser_analyzer_;
  History history_;
  rclcpp::Logger logger_ = get_logger();
};
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GofObstacleHuggingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
