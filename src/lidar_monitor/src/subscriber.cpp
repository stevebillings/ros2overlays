#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SubscriberNode : public rclcpp::Node {
	public:
		SubscriberNode() : Node("laser_monitor_node") {
            laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", 10,
                                                                                      std::bind(&SubscriberNode::callback, this, _1));
            // TODO once working, period should be 50ms
            timer_ = create_wall_timer(50ms, std::bind(&SubscriberNode::control_cycle, this));
            drive_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            set_state(STATE_SEARCH);
		}

        // TODO I think all of these can be private:
		void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            // TODO is this a concurrency-safe operation?
            laser_scan_msg_ = std::move(msg);
        }

        void control_cycle() {
            RCLCPP_INFO(get_logger(), "======================");
            RCLCPP_INFO(get_logger(), "State: %d (1 = hugging, 2 = recently lost", state_);
            if (laser_scan_msg_ == nullptr) {
                return;
            }
            interpret_laser(laser_scan_msg_);
            evaluate_obstacle_last_seen();

            switch (state_) {
                case STATE_SEARCH:
                    if (is_obstacle_within_range()) {
                        set_state(STATE_HUGGING);
                    } else {
                        if (obstacle_direction_ == DIR_AHEAD) {
                            drive_straight();
                        } else {
                            spin();
                        }
                    }
                    break;
                case STATE_HUGGING:
                    if (!is_obstacle_within_range()) {
                        set_state(STATE_RECENTLY_LOST);
                    } else if (ok_to_drive()) {
                        RCLCPP_INFO(get_logger(), "Go straight");
                        drive_straight();
                    } else {
                        RCLCPP_INFO(get_logger(), "Spin");
                        spin();
                    }
                    break;
                case STATE_RECENTLY_LOST:
                    if (is_obstacle_within_range()) {
                        set_state(STATE_HUGGING);
                    } else if (get_seconds_in_state() > 2.0) {
                        set_state(STATE_SEARCH);
                    } else if (obstacle_last_seen_ == OBSTACLE_LAST_SEEN_RIGHT) {
                        curve_right();
                    } else {
                        curve_left();
                    }
                    break;
                default:
                    RCLCPP_WARN(get_logger(), "Invalid state: %d", state_);
                    set_state(STATE_SEARCH);
            }
        }

        double get_seconds_in_state() {
            rclcpp::Time right_now = now();
            rclcpp::Duration time_in_state = right_now - state_ts_;
            double seconds_in_state = time_in_state.seconds();
            RCLCPP_INFO(get_logger(), "Seconds since entered current state: %lf", seconds_in_state);
            return seconds_in_state;
        }

        // TODO this should not handle multiple states!
        bool ok_to_drive() {
            bool ok = false;
            if (state_ == STATE_SEARCH) {
                ok = !is_obstacle_too_near();
            } else {
                if (is_obstacle_far()) {
                    ok = false;
                } else {
                    ok = !is_obstacle_too_near();
                }
            }
            return ok;
        }


        bool is_obstacle_within_range() {
            return ((ahead_dist_ == DIST_NAME_NEAR || ahead_dist_ == DIST_NAME_IDEAL)
                    || (left_dist_ == DIST_NAME_NEAR || left_dist_ == DIST_NAME_IDEAL)
                    || (right_dist_ == DIST_NAME_NEAR || right_dist_ == DIST_NAME_IDEAL));
        }

        bool is_obstacle_too_near() {
            return ((ahead_dist_ == DIST_NAME_NEAR)
                    || (left_dist_ == DIST_NAME_NEAR)
                    || (right_dist_ == DIST_NAME_NEAR));
        }

        bool is_obstacle_far() {
            return ((ahead_dist_ == DIST_NAME_FAR)
                    && (left_dist_ == DIST_NAME_FAR)
                    && (right_dist_ == DIST_NAME_FAR));
        }

        bool is_obstacle_distance_ideal() {
            bool one_is_ideal = ((ahead_dist_ == DIST_NAME_IDEAL)
                    || (left_dist_ == DIST_NAME_IDEAL)
                    || (right_dist_ == DIST_NAME_IDEAL));

            return one_is_ideal && !is_obstacle_too_near();
        }

        void interpret_laser(sensor_msgs::msg::LaserScan::SharedPtr msg) {
			/*
            RCLCPP_INFO(get_logger(), "Received LaserScan angle_min: %f; angle_max: %f; angle_increment: %f",
				msg->angle_min, msg->angle_max,
				msg->angle_increment);
			 */
			int i = 0;
			auto min_range = 999.999;
			int min_range_index = -1;
			for (auto this_range : msg->ranges) {
				if (this_range < min_range) {
					min_range = this_range;
					min_range_index = i;
				}
				i++;
			}
			// RCLCPP_INFO(get_logger(), "\tfound min range %f at index %d", min_range, min_range_index);
            int dead_ahead = msg->ranges.size() / 2;
            int ahead_width = msg->ranges.size() / 10;
            int ahead_width_from_center = ahead_width / 2;
            int right_limit = dead_ahead - ahead_width_from_center;
            int left_limit = dead_ahead + ahead_width_from_center;
            // RCLCPP_INFO(get_logger(), "Ahead is %d to %d", right_limit, left_limit);
            if (min_range < OK_LIMIT) {
                if (min_range_index < right_limit) {
                    RCLCPP_INFO(get_logger(), "Obstacle on right: %f meters", min_range);
                    if (min_range < NEAR_LIMIT) {
                        right_dist_ = DIST_NAME_NEAR;
                    } else {
                        right_dist_ = DIST_NAME_IDEAL;
                    }
                    left_dist_ = DIST_NAME_FAR;
                    ahead_dist_ = DIST_NAME_FAR;
                } else if (min_range_index > left_limit) {
                    RCLCPP_INFO(get_logger(), "Obstacle on left: %f meters", min_range);
                    if (min_range < NEAR_LIMIT) {
                        left_dist_ = DIST_NAME_NEAR;
                    } else {
                        left_dist_ = DIST_NAME_IDEAL;
                    }
                    right_dist_ = DIST_NAME_FAR;
                    ahead_dist_ = DIST_NAME_FAR;
                } else {
                    RCLCPP_INFO(get_logger(), "Obstacle straight ahead: %f meters", min_range);
                    if (min_range < NEAR_LIMIT) {
                        ahead_dist_ = DIST_NAME_NEAR;
                    } else {
                        ahead_dist_ = DIST_NAME_IDEAL;
                    }
                    right_dist_ = DIST_NAME_FAR;
                    left_dist_ = DIST_NAME_FAR;
                }
            } else {
                RCLCPP_INFO(get_logger(), "Clear");
                right_dist_ = DIST_NAME_FAR;
                left_dist_ = DIST_NAME_FAR;
                ahead_dist_ = DIST_NAME_FAR;

                if (min_range_index < right_limit) {
                    RCLCPP_INFO(get_logger(), "Obstacle on right: %f meters", min_range);
                    obstacle_direction_ = DIR_RIGHT;
                } else if (min_range_index > left_limit) {
                    RCLCPP_INFO(get_logger(), "Obstacle on left: %f meters", min_range);
                    obstacle_direction_ = DIR_LEFT;
                } else {
                    RCLCPP_INFO(get_logger(), "Obstacle straight ahead: %f meters", min_range);
                    obstacle_direction_ = DIR_AHEAD;
                }
            }
            RCLCPP_INFO(get_logger(), "left: %d; right: %d: ahead: %d (0=near, 1=ok, 2=far)", left_dist_, right_dist_, ahead_dist_);
		}

        void evaluate_obstacle_last_seen() {
            if (right_dist_ == DIST_NAME_IDEAL) {
                obstacle_last_seen_ = OBSTACLE_LAST_SEEN_RIGHT;
            } else if (left_dist_ == DIST_NAME_IDEAL) {
                obstacle_last_seen_ = OBSTACLE_LAST_SEEN_LEFT;
            }
            if (obstacle_last_seen_ == OBSTACLE_LAST_SEEN_RIGHT) {
                RCLCPP_INFO(get_logger(), "Obstacle was last seen on the right");
            } else if (obstacle_last_seen_ == OBSTACLE_LAST_SEEN_LEFT) {
                RCLCPP_INFO(get_logger(), "Obstacle was last seen on the left");
            }
        }

        void drive_straight() {
            RCLCPP_INFO(get_logger(), "Driving straight ahead");
            drive_message_.linear.x = SPEED; // ahead
            drive_message_.linear.y = 0.0;
            drive_message_.linear.z = 0.0;
            drive_message_.angular.x = 0.0;
            drive_message_.angular.y = 0.0;
            drive_message_.angular.z = 0.0; // yaw
            drive_publisher_->publish(drive_message_);
        }

        void curve_right() {
            RCLCPP_INFO(get_logger(), "Driving straight ahead");
            drive_message_.linear.x = SPEED; // ahead
            drive_message_.linear.y = 0.0;
            drive_message_.linear.z = 0.0;
            drive_message_.angular.x = 0.0;
            drive_message_.angular.y = 0.0;
            drive_message_.angular.z = -1 * SPEED / 5.0; // yaw
            drive_publisher_->publish(drive_message_);
        }

        void curve_left() {
            RCLCPP_INFO(get_logger(), "Driving straight ahead");
            drive_message_.linear.x = SPEED; // ahead
            drive_message_.linear.y = 0.0;
            drive_message_.linear.z = 0.0;
            drive_message_.angular.x = 0.0;
            drive_message_.angular.y = 0.0;
            drive_message_.angular.z = SPEED / 5.0; // yaw
            drive_publisher_->publish(drive_message_);
        }

        void spin() {
            float yaw = SPEED / 4.0; // default to spin left
            if (obstacle_last_seen_ != OBSTACLE_LAST_SEEN_NOWHERE) {
                if ((obstacle_last_seen_ == OBSTACLE_LAST_SEEN_RIGHT) && (right_dist_ == DIST_NAME_FAR)) {
                    yaw = -1.0 * yaw; // switch to spin right
                }
            }
            drive_message_.linear.x = 0.0; // ahead
            drive_message_.linear.y = 0.0;
            drive_message_.linear.z = 0.0;
            drive_message_.angular.x = 0.0;
            drive_message_.angular.y = 0.0;
            drive_message_.angular.z = yaw; // yaw
            drive_publisher_->publish(drive_message_);
        }

        void set_state(int new_state) {
            state_ = new_state;
            state_ts_ = now();
            RCLCPP_INFO(get_logger(), "Entering state %d at nanoseconds %ld", state_, state_ts_.nanoseconds());
        }
	private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
        geometry_msgs::msg::Twist drive_message_;
        rclcpp::Time state_ts_;
        static constexpr float SPEED = 1.0;
        static const int DIR_UNKNOWN = 0;
        static const int DIR_AHEAD = 1;
        static const int DIR_LEFT = 2;
        static const int DIR_RIGHT = 3;
        static const int OBSTACLE_LAST_SEEN_NOWHERE = 0;
        static const int OBSTACLE_LAST_SEEN_LEFT = 1;
        static const int OBSTACLE_LAST_SEEN_RIGHT = 2;
        static const int OK_LIMIT = 5.0;
        static const int NEAR_LIMIT = 3.0;
        static const int DIST_NAME_NEAR = 0;
        static const int DIST_NAME_IDEAL = 1;
        static const int DIST_NAME_FAR = 2;
        int left_dist_ = DIST_NAME_FAR;
        int right_dist_ = DIST_NAME_FAR;
        int ahead_dist_ = DIST_NAME_FAR;
        int obstacle_last_seen_ = OBSTACLE_LAST_SEEN_NOWHERE;
        static const int STATE_SEARCH = 0;
        static const int STATE_HUGGING = 1;
        static const int STATE_RECENTLY_LOST = 2;
        int state_;
        int obstacle_direction_ = DIR_UNKNOWN;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
