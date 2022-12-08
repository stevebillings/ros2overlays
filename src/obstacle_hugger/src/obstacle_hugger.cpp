#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "full_state.h"
#include "laser_analyzer.h"
#include "laser_analysis.h"
#include "velocity_calculator.h"

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
    {
      return;  // wait for sight
    }
    // TODO only get characteristics once
    LaserCharacteristics laser_characteristics = laser_analyzer_.determineCharacteristics(last_laser_scan_msg_);
    RCLCPP_INFO(logger_, "straight index: %ld, leftmost_index: %ld", laser_characteristics.getStraightIndex(),
                laser_characteristics.getLeftmostIndex());
    LaserAnalysis laser_analysis = laser_analyzer_.analyze(logger_, laser_characteristics, last_laser_scan_msg_);
    RCLCPP_INFO(logger_, "min_range_index: %ld; range: %lf; leftmost index: %ld",
                laser_analysis.getNearestSighting().getRangeIndex(),
                laser_analysis.getNearestSighting().getRange(), laser_characteristics.getLeftmostIndex());
    if (laser_analysis.isInSight())
    {
      setObstacleSeen(laser_analysis.isToRight());
      RCLCPP_INFO(logger_, "laser analysis: %s", laser_analysis.toString().c_str());
    }

    double time_lost = 0.0;
    if (full_state_.hasObstacleBeenSeen() && !laser_analysis.isInSight())
    {
      time_lost = now().seconds() - full_state_.getObstacleLastSeenTime();
    }

    switch (full_state_.getFsmState())
    {
      case FsmState::SEARCH:
        if (laser_analysis.isInSight())
        {
          Velocity approach_velocity = velocity_calculator_.toApproach(logger_, laser_characteristics, laser_analysis);
          RCLCPP_INFO(logger_, "Approaching: x: %lf; yaw: %lf", approach_velocity.getForward(),
                      approach_velocity.getYaw());
            setVelocity(approach_velocity);
          if (laser_analysis.isNear())
          {
              setState(FsmState::OBSTACLE_NEAR);
          }
        }
        else if (full_state_.hasObstacleBeenSeen() && !laser_analysis.isInSight()
          && time_lost > TIME_LOST_TOLERANCE_SECONDS)
        {
          RCLCPP_WARN(logger_, "We've lost track of the obstacle for more than %lf seconds", TIME_LOST_TOLERANCE_SECONDS);
          if (full_state_.wasObstacleLastSeenToRight())
          {
              setVelocity(Velocity::createSearchSpinRight());
          }
          else
          {
              setVelocity(Velocity::createSearchSpinLeft());
          }
        }
        else if (!full_state_.hasObstacleBeenSeen())
        {
          RCLCPP_WARN(logger_, "Obstacle has never been seen and is not within sight");
          if (full_state_.hasObstacleBeenSeen() && full_state_.wasObstacleLastSeenToRight())
          {
              setVelocity(Velocity::createSearchSpinRight());
          }
          else
          {
              setVelocity(Velocity::createSearchSpinLeft());
          }
        }
        break;
      case FsmState::OBSTACLE_NEAR:
        if (laser_analysis.isTooNear())
        {
            setVelocity(Velocity::createReverse());
            setState(FsmState::OBSTACLE_TOO_NEAR);
        }
        else if (laser_analysis.isInSight())
        {
          Velocity parallel_velocity = velocity_calculator_.toParallel(logger_, laser_analysis);
          RCLCPP_INFO(logger_, "Paralleling: x: %lf; yaw: %lf", parallel_velocity.getForward(),
                      parallel_velocity.getYaw());
            setVelocity(parallel_velocity);
        }
        else
        {
          RCLCPP_INFO(logger_, "Lost sight of obstacle %lf seconds ago", time_lost);
          if (time_lost < 1.0)
          {
            RCLCPP_INFO(logger_, "Lost sight of obstacle, but it's only been %lf seconds, so being patient...",
                        time_lost);
          }
          else
          {
            RCLCPP_WARN(logger_, "Obstacle is not within sight");
              setVelocity(Velocity::createStopped());
              setState(FsmState::SEARCH);
          }
        }
        break;
      case FsmState::OBSTACLE_TOO_NEAR:
        if (getSecondsInState() > 2.0)
        {
            setVelocity(Velocity::createStopped());
            setState(FsmState::SEARCH);
        }
        break;
      case FsmState::ERROR:
          setVelocity(Velocity::createStopped());
        break;
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
    if ((abs(velocity.getForward()) > 5.0) || (abs(velocity.getYaw()) > 5.0))
    {
      RCLCPP_ERROR(logger_, "Invalid velocity: x: %lf, yaw: %lf", velocity.getForward(), velocity.getYaw());
        setState(FsmState::ERROR);
      return;
    }
    geometry_msgs::msg::Twist drive_message;
    drive_message.linear.x = velocity.getForward();
    drive_message.angular.z = velocity.getYaw();
    drive_publisher_->publish(drive_message);
  }

  void setState(const FsmState& new_state)
  {
      full_state_.setState(new_state, now().seconds());
    RCLCPP_INFO(logger_, "New FsmState: %s", full_state_.getFsmStateName());
  }

  void setObstacleSeen(bool seen_to_right)
  {
      full_state_.setObstacleLastSeenTime(logger_, now().seconds(), seen_to_right);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
  LaserAnalyzer laser_analyzer_;
  FullState full_state_ = FullState();
  rclcpp::Logger logger_ = get_logger();
  VelocityCalculator velocity_calculator_ = VelocityCalculator();
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
