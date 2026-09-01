#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "slam_toolbox/initial_pose_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace slam_toolbox
{
namespace
{

constexpr double kHalfPi = 1.5707963267948966;

geometry_msgs::msg::PoseWithCovarianceStamped makePoseMessage(
  const std::string & frame_id,
  double x,
  double y,
  double yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame_id;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  msg.pose.pose.orientation = tf2::toMsg(orientation);
  return msg;
}

TEST(InitialPoseUtilsTests, LeavesPoseUntouchedInTargetFrame)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  const auto msg = makePoseMessage("map", 1.25, -0.5, 0.3);

  geometry_msgs::msg::PoseStamped pose_out;
  EXPECT_TRUE(
    transformInitialPoseToFrame(
      msg, "map", buffer, rclcpp::get_logger("initial_pose_utils_test"), "test", pose_out));
  EXPECT_DOUBLE_EQ(pose_out.pose.position.x, msg.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(pose_out.pose.position.y, msg.pose.pose.position.y);
  EXPECT_NEAR(
    tf2::getYaw(pose_out.pose.orientation),
    tf2::getYaw(msg.pose.pose.orientation),
    1e-9);
}

TEST(InitialPoseUtilsTests, TransformsPoseIntoTargetFrame)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "world";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 0.0;
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, kHalfPi);
  transform.transform.rotation = tf2::toMsg(rotation);
  EXPECT_TRUE(buffer.setTransform(transform, "test", true));

  const auto msg = makePoseMessage("world", 1.0, 0.0, 0.0);

  geometry_msgs::msg::PoseStamped pose_out;
  EXPECT_TRUE(
    transformInitialPoseToFrame(
      msg, "map", buffer, rclcpp::get_logger("initial_pose_utils_test"), "test", pose_out));
  EXPECT_NEAR(pose_out.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(pose_out.pose.position.y, 3.0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(pose_out.pose.orientation), kHalfPi, 1e-9);
}

}  // namespace
}  // namespace slam_toolbox

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
