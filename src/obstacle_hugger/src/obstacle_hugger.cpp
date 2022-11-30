#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "FullState.h"
#include "LaserAnalyzer.h"
#include "LaserAnalysis.h"
#include "VelocityCalculator.h"

static constexpr float SPEED = 0.5;

static const double DIST_WITHIN_SIGHT = 8.0;
static const double DIST_NEAR = 3.0;
static const double SPEED_DIVISOR_FOR_SPIN_YAW = 4.0;
static const double MAX_SECS_TO_HOPE_FOR_REDISCOVERY = 2.0;

static const unsigned long index_delta_close_to_straight_ahead = 30;
static const unsigned long index_delta_straight_ahead = 5;

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

            // TODO:
            // Approaching: Tolerate losing sight for a while without changing state; maybe up to 1 sec? 0.5 sec?
            // Paralleling: If lose sight: maybe go straight for 1 sec; if still lost, then -> RECENTLY_LOST
            // RECENTLY_LOST: arc hard toward direction last saw it
            //      New states (just lost, recently lost)?
            // Maybe state is an object:
            //      State
            //      Time entered state
            //      Last seen direction boolean
            //      Time last seen

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
                    } else {
                        RCLCPP_WARN(logger_, "Obstacle is not within sight");
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
                        // Lost sight of obstacle

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

        bool spotted_obstacle(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            unsigned long min_range_index = get_min_range_index(msg);
            double min_range = msg->ranges[min_range_index];
            if (min_range < DIST_WITHIN_SIGHT) {
                RCLCPP_INFO(logger_, "Obstacle is within sight (range index: %ld)", min_range_index);
                return true;
            }
            return false;
        }

        // returns -1 (obstacle is on right), 0 (obstacle is straight-ish ahead), or +1 (obstacle is on left)
        int obstacle_dir(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            unsigned long index_dead_ahead = msg->ranges.size() / 2;
            unsigned long min_range_index = get_min_range_index(msg);
            if (min_range_index < (index_dead_ahead - index_delta_straight_ahead)) {
                RCLCPP_INFO(logger_, "Obstacle is on the right");
                return -1; // right
            } else if (min_range_index > (index_dead_ahead + index_delta_straight_ahead)) {
                RCLCPP_INFO(logger_, "Obstacle is on the left");
                return 1; // left
            } else {
                RCLCPP_INFO(logger_, "Obstacle is straight ahead");
                return 0; // ahead
            }
        }

        bool obstacle_near(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            unsigned long min_range_index = get_min_range_index(msg);
            double min_range = msg->ranges[min_range_index];
            if (min_range < DIST_NEAR) {
                RCLCPP_INFO(logger_, "Obstacle is dangerously close");
                return true;
            }
            return false;
        }

        unsigned long get_min_range_index(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            unsigned long index_dead_ahead = msg->ranges.size() / 2;
            int cur_range_index = 0;
            double min_range = std::numeric_limits<double>::max();
            unsigned long min_range_index = index_dead_ahead;
            for (auto this_range : msg->ranges) {
                if (this_range < min_range) {
                    min_range = this_range;
                    min_range_index = cur_range_index;
                }
                cur_range_index++;
            }
            RCLCPP_INFO(logger_, "Spotted obstacle at range %lf at index %ld", min_range, min_range_index);
            return min_range_index;
        }

        void drive_straight() {
            RCLCPP_INFO(logger_, "Driving straight ahead");
            geometry_msgs::msg::Twist drive_message;
            drive_message.linear.x = SPEED; // ahead
            drive_message.angular.z = 0.0;
            drive_publisher_->publish(drive_message);
        }

        int calc_spin_velo_mult_search(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            unsigned long index_dead_ahead = msg->ranges.size() / 2;
            return calc_spin_velo_mult(msg, index_dead_ahead);
        }

        int calc_spin_velo_mult_near(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            unsigned long index_ninety_degrees = index_delta_close_to_straight_ahead / 2;
            return calc_spin_velo_mult(msg, index_ninety_degrees);
        }

        int calc_spin_velo_mult(sensor_msgs::msg::LaserScan::SharedPtr msg, unsigned long target_dir_index) {
                // TODO: smooth this out using a single equation

            if (!spotted_obstacle(msg)) {
                return 10;
            }
            unsigned long min_range_index = get_min_range_index(msg);
            RCLCPP_INFO(logger_, "===> Obstacle found at index %ld; target_dir_index: %ld",
                        min_range_index, target_dir_index);

            RCLCPP_INFO(logger_, "Comparing index %ld to limit %ld", min_range_index,
                        (target_dir_index - index_delta_close_to_straight_ahead));

            if ((min_range_index > (target_dir_index - index_delta_straight_ahead))
                && (min_range_index < (target_dir_index + index_delta_straight_ahead))) {
                RCLCPP_INFO(logger_, "Obstacle is very close to the target_dir_index %ld", target_dir_index);
                return 0; // dead ahead
            } else if (min_range_index < (target_dir_index - index_delta_close_to_straight_ahead)) {
                RCLCPP_INFO(logger_, "Obstacle is far right of the target index %ld", target_dir_index);
                return 2; // in sight on right
            } else if (min_range_index > (target_dir_index + index_delta_close_to_straight_ahead)) {
                RCLCPP_INFO(logger_, "Obstacle is far left of the target index %ld", target_dir_index);
                return 2; // in sight on left
            } else {
                RCLCPP_INFO(logger_, "Obstacle is getting close to the target_dir_index %ld", target_dir_index);
                return 1; // getting close to, but not at, dead ahead
            }
        }

        void spin(int spin_dir, int spin_velocity_multiplier) {
            float yaw = spin_velocity_multiplier * spin_dir * (SPEED / SPEED_DIVISOR_FOR_SPIN_YAW);
            geometry_msgs::msg::Twist drive_message;
            drive_message.linear.x = 0.0;
            drive_message.angular.z = yaw; // yaw
            RCLCPP_INFO(logger_, "Spinning with spin_dir %d; yaw %f", spin_dir, yaw);
            drive_publisher_->publish(drive_message);
        }

        void stop() {
            RCLCPP_INFO(logger_, "Stopping");
            geometry_msgs::msg::Twist drive_message;
            drive_message.linear.x = 0.0;
            drive_message.angular.z = 0.0;
            drive_publisher_->publish(drive_message);
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
