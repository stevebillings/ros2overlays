#include <gtest/gtest.h>
#include "../src/laser/laser_analyzer.h"

// A mock packet stream class.  It inherits from no other, but defines
// GetPacket() and NumberOfPackets().
/*
class MockLogger {
public:
  MOCK_METHOD(const Packet*, GetPacket, (size_t packet_number), (const));
  MOCK_METHOD(size_t, NumberOfPackets, (), (const));
  ...
};
 */

TEST(LaserTest, CrazySimple) {
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(7*6, 42);
}

TEST(LaserTest, FirstTest) {

  // const rclcpp::Logger& logger, const LaserCharacteristics& laserCharacteristics,
  //                        sensor_msgs::msg::LaserScan::SharedPtr msg
//  rclcpp::Logger logger;
//  LaserAnalyzer laserAnalyzer;
//  laserAnalyzer.analyze();
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(7*6, 42);
}