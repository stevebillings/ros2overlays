#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "full_state.h"
#include "../laser/laser_analyzer.h"
#include "../velocity/velocity_calculator.h"
#include "../history/history.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObstacleHuggingNode : public rclcpp::Node
{
public:
  ObstacleHuggingNode() : Node("obstacle_hugging_node")
  {
    laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "laser_scan", 10, std::bind(&ObstacleHuggingNode::laserScanCallback, this, _1));
    timer_ = create_wall_timer(50ms, std::bind(&ObstacleHuggingNode::controlCallback, this));
    drive_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      setState(FsmState::SEARCH);
  }

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_laser_scan_msg_ = std::move(msg);
  }

  void controlCallback()
  {
    RCLCPP_INFO(logger_, "======================");
    RCLCPP_INFO(logger_, "* FsmState: %s", full_state_.getFsmStateName());
    if (last_laser_scan_msg_ == nullptr)
      return;  // ain't seen nothin' yet

    init_laser_characteristics();
    LaserAnalysis laser_analysis = laser_analyzer_.analyze(*laser_characteristics_, last_laser_scan_msg_->ranges);

    // TODO: These to data points are new observations; maybe pass them and laser characteristics + analysis to state.act(observations)
    // They are both time-related; time since X. ***history***
    // TODO should time_lost be stored in state? Not sure new info belongs in state
    if (laser_analysis.isInSight())
    {
      bool seenToRight = laser_analysis.isToRight();
      history_.set_obstacle_last_seen_time(now().seconds(), seenToRight);
      // TODO obsolete:
      full_state_.setObstacleLastSeenTime(now().seconds(), seenToRight);
    }
    double time_lost = calculate_time_lost(laser_analysis);
    history_.set_time_lost(time_lost);

    switch (full_state_.getFsmState())
    {
      case FsmState::SEARCH:
        if (laser_analysis.isInSight())
        {
          Velocity approach_velocity = velocity_calculator_.toApproach(*laser_characteristics_, laser_analysis);
          RCLCPP_INFO(logger_, "Approaching: x: %lf; yaw: %lf", approach_velocity.get_forward(),
                      approach_velocity.get_yaw());
            setVelocity(approach_velocity);
          if (laser_analysis.isNear())
          {
              setState(FsmState::OBSTACLE_NEAR);
          }
        }
        else if (full_state_.hasObstacleBeenSeen() && !laser_analysis.isInSight()
          && history_.get_time_lost() > TIME_LOST_TOLERANCE_SECONDS)
        {
          RCLCPP_WARN(logger_, "We've lost track of the obstacle for more than %lf seconds", TIME_LOST_TOLERANCE_SECONDS);
          if (full_state_.wasObstacleLastSeenToRight())
          {
              setVelocity(Velocity::create_spin_right());
          }
          else
          {
              setVelocity(Velocity::create_spin_left());
          }
        }
        else if (!full_state_.hasObstacleBeenSeen())
        {
          RCLCPP_WARN(logger_, "Obstacle has never been seen and is not within sight");
          if (full_state_.hasObstacleBeenSeen() && full_state_.wasObstacleLastSeenToRight())
          {
              setVelocity(Velocity::create_spin_right());
          }
          else
          {
              setVelocity(Velocity::create_spin_left());
          }
        }
        break;
      case FsmState::OBSTACLE_NEAR:
        if (laser_analysis.isTooNear())
        {
            setVelocity(Velocity::create_reverse());
            setState(FsmState::OBSTACLE_TOO_NEAR);
        }
        else if (laser_analysis.isInSight())
        {
          Velocity parallel_velocity = velocity_calculator_.toParallel(laser_analysis);
          RCLCPP_INFO(logger_, "Paralleling: x: %lf; yaw: %lf", parallel_velocity.get_forward(),
                      parallel_velocity.get_yaw());
            setVelocity(parallel_velocity);
        }
        else
        {
          RCLCPP_INFO(logger_, "Lost sight of obstacle %lf seconds ago", history_.get_time_lost());
          if (history_.get_time_lost() < 1.0)
          {
            RCLCPP_INFO(logger_, "Lost sight of obstacle, but it's only been %lf seconds, so being patient...",
                        history_.get_time_lost());
          }
          else
          {
            RCLCPP_WARN(logger_, "Obstacle is not within sight");
              setVelocity(Velocity::create_stopped());
              setState(FsmState::SEARCH);
          }
        }
        break;
      case FsmState::OBSTACLE_TOO_NEAR:
        if (getSecondsInState() > 2.0)
        {
            setVelocity(Velocity::create_stopped());
            setState(FsmState::SEARCH);
        }
        break;
      case FsmState::ERROR:
          setVelocity(Velocity::create_stopped());
        break;
    }
  }

  double calculate_time_lost(const LaserAnalysis &laser_analysis) const
  {
    double time_lost = 0.0;
    if (full_state_.hasObstacleBeenSeen() && !laser_analysis.isInSight())
    {
      time_lost = now().seconds() - full_state_.getObstacleLastSeenTime();
    }
    return time_lost;
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

  double getSecondsInState() const
  {
    double seconds_in_state = now().seconds() - full_state_.getStateStartTime();
    RCLCPP_INFO(logger_, "Seconds since entered current state: %lf", seconds_in_state);
    return seconds_in_state;
  }

  void setVelocity(const Velocity& velocity)
  {
    if ((abs(velocity.get_forward()) > 5.0) || (abs(velocity.get_yaw()) > 5.0))
    {
      RCLCPP_ERROR(logger_, "Invalid velocity: x: %lf, yaw: %lf", velocity.get_forward(), velocity.get_yaw());
        setState(FsmState::ERROR);
      return;
    }
    geometry_msgs::msg::Twist drive_message;
    drive_message.linear.x = velocity.get_forward();
    drive_message.angular.z = velocity.get_yaw();
    drive_publisher_->publish(drive_message);
  }

  void setState(const FsmState& new_state)
  {
      full_state_.setState(new_state, now().seconds());
    RCLCPP_INFO(logger_, "New FsmState: %s", full_state_.getFsmStateName());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
  LaserAnalyzer laser_analyzer_;
  FullState full_state_ = FullState();
  rclcpp::Logger logger_ = get_logger();
  VelocityCalculator velocity_calculator_ = VelocityCalculator();
  LaserCharacteristics* laser_characteristics_ = nullptr;
  History history_ = History();
  static constexpr double TIME_LOST_TOLERANCE_SECONDS = 0.75;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleHuggingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
