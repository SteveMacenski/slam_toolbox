// Copyright 2026 KR-Ravindra
// Regression test for map_start_at_dock being ignored when map_start_pose is unset (#842).

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/slam_toolbox_localization.hpp"

class TestableLocalization : public slam_toolbox::LocalizationSlamToolbox
{
public:
  explicit TestableLocalization(rclcpp::NodeOptions o)
  : LocalizationSlamToolbox(o) {}
  using slam_toolbox::SlamToolbox::shouldStartWithPoseGraph;
};

static std::shared_ptr<TestableLocalization> makeNode(
  const std::vector<rclcpp::Parameter> & overrides)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides(overrides);
  return std::make_shared<TestableLocalization>(o);
}

// Variant 3 of #843: map_start_at_dock=true, map_start_pose unset -> dock honoured.
TEST(MapStartAtDock, DockHonouredWhenPoseUnset)
{
  auto node = makeNode({
      rclcpp::Parameter("map_file_name", "/tmp/some_map.posegraph"),
      rclcpp::Parameter("map_start_at_dock", true)});
  std::string fn;
  geometry_msgs::msg::Pose2D pose;
  bool dock = false;
  EXPECT_TRUE(node->shouldStartWithPoseGraph(fn, pose, dock));
  EXPECT_TRUE(dock) << "map_start_at_dock: true was ignored (#842)";
  EXPECT_EQ(fn, "/tmp/some_map.posegraph");
}

// Variant 1 of #843: neither given -> error, do not start with pose graph.
TEST(MapStartAtDock, NeitherGivenReturnsFalse)
{
  auto node = makeNode({rclcpp::Parameter("map_file_name", "/tmp/some_map.posegraph")});
  std::string fn;
  geometry_msgs::msg::Pose2D pose;
  bool dock = false;
  EXPECT_FALSE(node->shouldStartWithPoseGraph(fn, pose, dock));
  EXPECT_FALSE(dock);
}

// Variant 2 of #843: valid pose wins over dock.
TEST(MapStartAtDock, PoseWinsOverDock)
{
  auto node = makeNode({
      rclcpp::Parameter("map_file_name", "/tmp/some_map.posegraph"),
      rclcpp::Parameter("map_start_at_dock", true),
      rclcpp::Parameter("map_start_pose", std::vector<double>{1.0, 2.0, 0.5})});
  std::string fn;
  geometry_msgs::msg::Pose2D pose;
  bool dock = true;
  EXPECT_TRUE(node->shouldStartWithPoseGraph(fn, pose, dock));
  EXPECT_FALSE(dock);
  EXPECT_DOUBLE_EQ(pose.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.theta, 0.5);
}

// Variant 4 of #843: malformed pose -> origin fallback, still starts.
TEST(MapStartAtDock, MalformedPoseFallsBackToOrigin)
{
  auto node = makeNode({
      rclcpp::Parameter("map_file_name", "/tmp/some_map.posegraph"),
      rclcpp::Parameter("map_start_pose", std::vector<double>{1.0, 2.0})});
  std::string fn;
  geometry_msgs::msg::Pose2D pose;
  bool dock = true;
  EXPECT_TRUE(node->shouldStartWithPoseGraph(fn, pose, dock));
  EXPECT_FALSE(dock);
  EXPECT_DOUBLE_EQ(pose.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.theta, 0.0);
}

TEST(MapStartAtDock, NoMapFileReturnsFalse)
{
  auto node = makeNode({rclcpp::Parameter("map_start_at_dock", true)});
  std::string fn;
  geometry_msgs::msg::Pose2D pose;
  bool dock = false;
  EXPECT_FALSE(node->shouldStartWithPoseGraph(fn, pose, dock));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
