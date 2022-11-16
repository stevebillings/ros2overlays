#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

static const int STATE_SEARCH = 0;
static const int STATE_HUGGING = 1;
static const int STATE_RECENTLY_LOST = 2;
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
static const double SPEED_DIVISOR_FOR_SPIN_YAW = 4.0;
static const double SPEED_DIVISOR_FOR_CURVE_YAW = 5.0;
static const double MAX_SECS_TO_HOPE_FOR_REDISCOVERY = 2.0;
using std::placeholders::_1;
using namespace std::chrono_literals;

class SubscriberNode : public rclcpp::Node {
	public:
		SubscriberNode() : Node("laser_monitor_node") {
            laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", 10,
                                                                                      std::bind(
                                                                                              &SubscriberNode::laser_scan_callback, this, _1));
            // TODO once working, period should be 50ms
            timer_ = create_wall_timer(50ms, std::bind(&SubscriberNode::control_callback, this));
            drive_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            set_state(STATE_SEARCH);
		}

    private:
		void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            last_laser_scan_msg_ = std::move(msg);
        }

        void control_callback() {
            RCLCPP_INFO(get_logger(), "======================");
            RCLCPP_INFO(get_logger(), "State: %d (0 = search; 1 = hugging, 2 = recently lost", state_);
            if (last_laser_scan_msg_ == nullptr) {
                return; // wait for sight
            }
            interpret_laser(last_laser_scan_msg_);
            evaluate_obstacle_last_seen();

            switch (state_) {
                case STATE_SEARCH:
                    if (is_obstacle_within_range()) {
                        set_state(STATE_HUGGING);
                    } else {
                        if (is_obstacle_ahead()) {
                            drive_straight();
                        } else {
                            spin();
                        }
                    }
                    break;
                case STATE_HUGGING:
                    if (!is_obstacle_within_range()) {
                        set_state(STATE_RECENTLY_LOST);
                    } else {
                        if (is_obstacle_far() || is_obstacle_too_near()) {
                            spin();
                        } else {
                            drive_straight();
                        }
                    }
                    break;
                case STATE_RECENTLY_LOST:
                    if (is_obstacle_within_range()) {
                        set_state(STATE_HUGGING);
                    } else if (get_seconds_in_state() > MAX_SECS_TO_HOPE_FOR_REDISCOVERY) {
                        set_state(STATE_SEARCH);
                    } else if (is_obstacle_on_right()) {
                        curve_right();
                    } else {
                        curve_left();
                    }
                    break;
            }
        }

        double get_seconds_in_state() {
            rclcpp::Time right_now = now();
            rclcpp::Duration time_in_state = right_now - state_start_time;
            double seconds_in_state = time_in_state.seconds();
            RCLCPP_INFO(get_logger(), "Seconds since entered current state: %lf", seconds_in_state);
            return seconds_in_state;
        }

        bool is_obstacle_ahead() {
            return obstacle_direction_ == DIR_AHEAD;
        }
        
        bool is_obstacle_on_right() {
            return obstacle_last_seen_ == OBSTACLE_LAST_SEEN_RIGHT;
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

        void interpret_laser(sensor_msgs::msg::LaserScan::SharedPtr msg) {
			int i = 0;
			auto min_range = 999.999;
			long min_range_index = -1;
			for (auto this_range : msg->ranges) {
				if (this_range < min_range) {
					min_range = this_range;
					min_range_index = i;
				}
				i++;
			}
            unsigned long dead_ahead = msg->ranges.size() / 2;
            unsigned long ahead_width = msg->ranges.size() / 10;
            unsigned long ahead_width_from_center = ahead_width / 2;
            unsigned long right_limit = dead_ahead - ahead_width_from_center;
            unsigned long left_limit = dead_ahead + ahead_width_from_center;
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
            geometry_msgs::msg::Twist drive_message;
            drive_message.linear.x = SPEED; // ahead
            drive_publisher_->publish(drive_message);
        }

        void curve_right() {
            RCLCPP_INFO(get_logger(), "Driving straight ahead");
            geometry_msgs::msg::Twist drive_message;
            drive_message.linear.x = SPEED; // ahead
            drive_message.angular.z = -1 * SPEED / SPEED_DIVISOR_FOR_CURVE_YAW;
            drive_publisher_->publish(drive_message);
        }

        void curve_left() {
            RCLCPP_INFO(get_logger(), "Driving straight ahead");
            geometry_msgs::msg::Twist drive_message;
            drive_message.linear.x = SPEED; // ahead
            drive_message.angular.z = SPEED / SPEED_DIVISOR_FOR_CURVE_YAW;
            drive_publisher_->publish(drive_message);
        }

        void spin() {
            float yaw = SPEED / SPEED_DIVISOR_FOR_SPIN_YAW; // default to spin left
            if ((obstacle_last_seen_ == OBSTACLE_LAST_SEEN_RIGHT) && (right_dist_ == DIST_NAME_FAR)) {
                yaw = -1.0 * yaw; // switch to spin right
            }
            geometry_msgs::msg::Twist drive_message;
            drive_message.angular.z = yaw; // yaw
            drive_publisher_->publish(drive_message);
        }

        void set_state(int new_state) {
            state_ = new_state;
            state_start_time = now();
            RCLCPP_INFO(get_logger(), "Entering state %d at nanoseconds %ld", state_, state_start_time.nanoseconds());
        }
	private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
        sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_msg_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
        rclcpp::Time state_start_time;

        int left_dist_ = DIST_NAME_FAR;
        int right_dist_ = DIST_NAME_FAR;
        int ahead_dist_ = DIST_NAME_FAR;
        int obstacle_last_seen_ = OBSTACLE_LAST_SEEN_NOWHERE;
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
