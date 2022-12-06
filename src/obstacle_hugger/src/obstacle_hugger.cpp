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
        "laser_scan", 10, std::bind(&ObstacleHuggingNode::laser_scan_callback, this, _1));
    timer_ = create_wall_timer(50ms, std::bind(&ObstacleHuggingNode::control_callback, this));
    drive_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    set_state(FsmState::SEARCH);
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_laser_scan_msg_ = std::move(msg);
  }

  void control_callback()
  {
    RCLCPP_INFO(logger_, "======================");
    RCLCPP_INFO(logger_, "* FsmState: %s", fullState_.get_fsm_state_name());
    if (last_laser_scan_msg_ == nullptr)
    {
      return;  // wait for sight
    }
    // TODO only get characteristics once
    LaserCharacteristics laserCharacteristics = laserAnalyzer_.determine_characteristics(last_laser_scan_msg_);
    RCLCPP_INFO(logger_, "straight index: %ld, leftmost_index: %ld", laserCharacteristics.get_straight_index(),
                laserCharacteristics.get_leftmost_index());
    LaserAnalysis laserAnalysis = laserAnalyzer_.analyze(logger_, laserCharacteristics, last_laser_scan_msg_);
    RCLCPP_INFO(logger_, "min_range_index: %ld; range: %lf; leftmost index: %ld",
                laserAnalysis.get_nearest_sighting().get_range_index(),
                laserAnalysis.get_nearest_sighting().get_range(), laserCharacteristics.get_leftmost_index());
    if (laserAnalysis.is_in_sight())
    {
      set_obstacle_seen(laserAnalysis.is_to_right());
      // TODO analysis class should have some sort of toString() to do this
      if (laserAnalysis.is_to_right())
      {
        RCLCPP_INFO(logger_, "%ld increments away from perpendicular right",
                    laserAnalysis.get_delta_from_perpendicular_right());
      }
      else
      {
        RCLCPP_INFO(logger_, "%ld increments away from perpendicular left",
                    laserAnalysis.get_delta_from_perpendicular_left());
      }
    }

    double time_lost = 0.0;
    if (fullState_.has_obstacle_been_seen() && !laserAnalysis.is_in_sight())
    {
      time_lost = now().seconds() - fullState_.get_obstacle_last_seen_time();
    }

    switch (fullState_.get_fsm_state())
    {
      case FsmState::SEARCH:
        if (laserAnalysis.is_in_sight())
        {
          Velocity approachVelocity = velocityCalculator_.toApproach(logger_, laserCharacteristics, laserAnalysis);
          RCLCPP_INFO(logger_, "Approaching: x: %lf; yaw: %lf", approachVelocity.get_forward(),
                      approachVelocity.get_yaw());
          set_velocity(approachVelocity);
          if (laserAnalysis.is_near())
          {
            set_state(FsmState::OBSTACLE_NEAR);
          }
        }
        else if (fullState_.has_obstacle_been_seen() && !laserAnalysis.is_in_sight() && time_lost > 1.0)
        {
          RCLCPP_WARN(logger_, "We've lost track of the obstacle for more than a second");
          if (fullState_.was_obstacle_last_seen_to_right())
          {
            set_velocity(Velocity::createSearchSpinRight());
          }
          else
          {
            set_velocity(Velocity::createSearchSpinLeft());
          }
        }
        else if (!fullState_.has_obstacle_been_seen())
        {
          RCLCPP_WARN(logger_, "Obstacle has never been seen and is not within sight");
          if (fullState_.has_obstacle_been_seen() && fullState_.was_obstacle_last_seen_to_right())
          {
            set_velocity(Velocity::createSearchSpinRight());
          }
          else
          {
            set_velocity(Velocity::createSearchSpinLeft());
          }
        }
        break;
      case FsmState::OBSTACLE_NEAR:
        if (laserAnalysis.is_too_near())
        {
          set_velocity(Velocity::createReverse());
          set_state(FsmState::OBSTACLE_TOO_NEAR);
        }
        else if (laserAnalysis.is_in_sight())
        {
          Velocity parallelVelocity = velocityCalculator_.toParallel(logger_, laserAnalysis);
          RCLCPP_INFO(logger_, "Paralleling: x: %lf; yaw: %lf", parallelVelocity.get_forward(),
                      parallelVelocity.get_yaw());
          set_velocity(parallelVelocity);
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
            set_velocity(Velocity::createStopped());
            set_state(FsmState::SEARCH);
          }
        }
        break;
      case FsmState::OBSTACLE_TOO_NEAR:
        if (get_seconds_in_state() > 2.0)
        {
          set_velocity(Velocity::createStopped());
          set_state(FsmState::SEARCH);
        }
        break;
      case FsmState::ERROR:
        set_velocity(Velocity::createStopped());
        break;
    }
  }

  double get_seconds_in_state() const
  {
    double seconds_in_state = now().seconds() - fullState_.get_state_start_time();
    RCLCPP_INFO(logger_, "Seconds since entered current state: %lf", seconds_in_state);
    return seconds_in_state;
  }

  void set_velocity(const Velocity& velocity)
  {
    // RCLCPP_INFO(logger_, "Setting new velocity: x: %lf, yaw: %lf", velocity.get_forward(), velocity.get_yaw());
    if ((abs(velocity.get_forward()) > 5.0) || (abs(velocity.get_yaw() > 5.0)))
    {
      RCLCPP_ERROR(logger_, "Invalid velocity: x: %lf, yaw: %lf", velocity.get_forward(), velocity.get_yaw());
      set_state(FsmState::ERROR);
      return;
    }
    geometry_msgs::msg::Twist drive_message;
    drive_message.linear.x = velocity.get_forward();
    drive_message.angular.z = velocity.get_yaw();
    drive_publisher_->publish(drive_message);
  }

  void set_state(const FsmState& new_state)
  {
    fullState_.set_state(new_state, now().seconds());
    RCLCPP_INFO(logger_, "New FsmState: %s", fullState_.get_fsm_state_name());
  }

  void set_obstacle_seen(bool seen_to_right)
  {
    fullState_.set_obstacle_last_seen_time(logger_, now().seconds(), seen_to_right);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
  LaserAnalyzer laserAnalyzer_;
  FullState fullState_ = FullState();
  rclcpp::Logger logger_ = get_logger();
  VelocityCalculator velocityCalculator_ = VelocityCalculator();
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleHuggingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
