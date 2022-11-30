#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "FullState.h"
#include "LaserAnalyzer.h"
#include "LaserAnalysis.h"
#include "VelocityCalculator.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObstacleHuggingNode : public rclcpp::Node {
	public:
		ObstacleHuggingNode() : Node("obstacle_hugging_node") {
            laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", 10,
                                                                                      std::bind(
                                                                                              &ObstacleHuggingNode::laser_scan_callback, this, _1));
            // TODO once working, period should be 50ms
            timer_ = create_wall_timer(50ms, std::bind(&ObstacleHuggingNode::control_callback, this));
            drive_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            set_state(State::SEARCH);
		}

    private:
		void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            last_laser_scan_msg_ = std::move(msg);
        }

        void control_callback() {
            RCLCPP_INFO(logger_, "======================");
            RCLCPP_INFO(logger_, "* State: %s", state_.get_name());
            if (last_laser_scan_msg_ == nullptr) {
                return; // wait for sight
            }
            LaserAnalysis laserAnalysis = laserAnalyzer_.analyze(last_laser_scan_msg_);
            RCLCPP_INFO(logger_, "min_range_index: %ld; range: %lf; leftmost index: %ld",
                        laserAnalysis.get_min_range_index(),
                        laserAnalysis.get_min_range(),
                        laserAnalysis.get_leftmost_index());
            if (laserAnalysis.is_in_sight()) {
                set_obstacle_seen(laserAnalysis.is_to_right());
                // TODO analysis class should have some sort of toString() to do this
                if (laserAnalysis.is_to_right()) {
                    RCLCPP_INFO(logger_, "%ld increments away from perpendicular right",
                                laserAnalysis.get_delta_from_perpendicular_right());
                } else {
                    RCLCPP_INFO(logger_, "%ld increments away from perpendicular left",
                                laserAnalysis.get_delta_from_perpendicular_left());
                }
            }
            VelocityCalculator velocityCalculator = VelocityCalculator();
            Velocity approachVelocity = velocityCalculator.toApproach(logger_, laserAnalysis);
            Velocity parallelVelocity = velocityCalculator.toParallel(logger_, laserAnalysis);
            double time_lost = 0.0;
            if (state_.has_obstacle_been_seen() && !laserAnalysis.is_in_sight()) {
                time_lost = now().seconds() - state_.get_obstacle_last_seen_time();
            }

            switch (state_.get_state()) {
                case State::SEARCH:
                    if (laserAnalysis.is_in_sight()) {
                        RCLCPP_INFO(logger_, "Approaching: x: %lf; yaw: %lf", approachVelocity.get_forward(),
                                    approachVelocity.get_yaw());
                        set_velocity(approachVelocity);
                        if (approachVelocity.get_forward() == 0.0) {
                            set_state(State::OBSTACLE_NEAR);
                        }
                    } else if (state_.has_obstacle_been_seen() && !laserAnalysis.is_in_sight() && time_lost > 1.0) {
                        RCLCPP_WARN(logger_, "We've lost track of the obstacle for more than a second");
                        if (state_.was_obstacle_last_seen_to_right()) {
                            set_velocity(Velocity::createSearchSpinRight());
                        } else {
                            set_velocity(Velocity::createSearchSpinLeft());
                        }
                    } else if (!state_.has_obstacle_been_seen()) {
                        RCLCPP_WARN(logger_, "Obstacle has never been seen and is not within sight");
                        if (state_.has_obstacle_been_seen() && state_.was_obstacle_last_seen_to_right()) {
                            set_velocity(Velocity::createSearchSpinRight());
                        } else {
                            set_velocity(Velocity::createSearchSpinLeft());
                        }
                    }
                    break;
                case State::OBSTACLE_NEAR:
                    if (laserAnalysis.is_too_near()) {
                        set_velocity(Velocity::createReverse());
                        set_state(State::OBSTACLE_TOO_NEAR);
                    } else if (laserAnalysis.is_in_sight()) {
                        RCLCPP_INFO(logger_, "Paralleling: x: %lf; yaw: %lf", parallelVelocity.get_forward(),
                                    parallelVelocity.get_yaw());
                        set_velocity(parallelVelocity);
                    } else {
                        RCLCPP_INFO(logger_, "Lost sight of obstacle %lf seconds ago", time_lost);
                        if (time_lost < 1.0) {
                            RCLCPP_INFO(logger_, "Lost sight of obstacle, but it's only been %lf seconds, so being patient...",
                                        time_lost);
                        } else {
                            RCLCPP_WARN(logger_, "Obstacle is not within sight");
                            set_velocity(Velocity::createStopped());
                            set_state(State::SEARCH);
                        }
                    }
                    break;
                case State::OBSTACLE_TOO_NEAR:
                    if (get_seconds_in_state() > 2.0) {
                        set_velocity(Velocity::createStopped());
                        set_state(State::SEARCH);
                    }
                    break;
            }
        }

        double get_seconds_in_state() {
            double right_now = now().seconds();
            double seconds_in_state = right_now - state_.get_state_start_time();
            RCLCPP_INFO(logger_, "Seconds since entered current state: %lf", seconds_in_state);
            return seconds_in_state;
        }

        void set_velocity(Velocity velocity) {
            RCLCPP_INFO(logger_, "Setting new velocity: x: %lf, yaw: %lf", velocity.get_forward(), velocity.get_yaw());
            geometry_msgs::msg::Twist drive_message;
            drive_message.linear.x = velocity.get_forward();
            drive_message.angular.z = velocity.get_yaw();
            drive_publisher_->publish(drive_message);
        }

        void set_state(State new_state) {
            state_.set_state(new_state, now().seconds());
            RCLCPP_INFO(logger_, "New State: %s", state_.get_name());
        }
        void set_obstacle_seen(bool seen_to_right) {
            state_.set_obstacle_last_seen_time(logger_, now().seconds(), seen_to_right);
        }
	private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
        sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_msg_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
        LaserAnalyzer laserAnalyzer_;
        FullState state_ = FullState();
        rclcpp::Logger logger_ = get_logger();
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleHuggingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
