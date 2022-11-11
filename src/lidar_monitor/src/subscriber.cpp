#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SubscriberNode : public rclcpp::Node {
	public:
		SubscriberNode() : Node("laser_monitor_node") {
            laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", 10,
                                                                                      std::bind(&SubscriberNode::callback, this, _1));
            timer_ = create_wall_timer(50ms, std::bind(&SubscriberNode::control_cycle, this));
		}

		void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            laser_scan_msg_ = std::move(msg);
        }

        void control_cycle() {
            if (laser_scan_msg_ == nullptr) {
                return;
            }
            interpret_laser(laser_scan_msg_);
            if (ahead_dist_ == DIST_NAME_FAR || ahead_dist_ == DIST_NAME_OK) {
                RCLCPP_INFO(get_logger(), "Go straight");
            } else {
                RCLCPP_INFO(get_logger(), "Stop");
            }
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
                        right_dist_ = DIST_NAME_OK;
                    }
                    left_dist_ = DIST_NAME_FAR;
                    ahead_dist_ = DIST_NAME_FAR;
                } else if (min_range_index > left_limit) {
                    RCLCPP_INFO(get_logger(), "Obstacle on left: %f meters", min_range);
                    if (min_range < NEAR_LIMIT) {
                        left_dist_ = DIST_NAME_NEAR;
                    } else {
                        left_dist_ = DIST_NAME_OK;
                    }
                    right_dist_ = DIST_NAME_FAR;
                    ahead_dist_ = DIST_NAME_FAR;
                } else {
                    RCLCPP_INFO(get_logger(), "Obstacle straight ahead: %f meters", min_range);
                    if (min_range < NEAR_LIMIT) {
                        ahead_dist_ = DIST_NAME_NEAR;
                    } else {
                        ahead_dist_ = DIST_NAME_OK;
                    }
                    right_dist_ = DIST_NAME_FAR;
                    left_dist_ = DIST_NAME_FAR;
                }
            } else {
                RCLCPP_INFO(get_logger(), "Clear");
                right_dist_ = DIST_NAME_FAR;
                left_dist_ = DIST_NAME_FAR;
                ahead_dist_ = DIST_NAME_FAR;
            }
            RCLCPP_INFO(get_logger(), "left: %d; right: %d: ahead: %d (0=near, 1=ok, 2=far)", left_dist_, right_dist_, ahead_dist_);
		}

	private:
        static const int OK_LIMIT = 4.0;
        static const int NEAR_LIMIT = 2.0;
        static const int DIST_NAME_NEAR = 0;
        static const int DIST_NAME_OK = 1;
        static const int DIST_NAME_FAR = 2;
        int left_dist_ = DIST_NAME_FAR;
        int right_dist_ = DIST_NAME_FAR;
        int ahead_dist_ = DIST_NAME_FAR;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
