// Copyright 2026 Sabeeh Saad
// SPDX-License-Identifier: LGPL-2.1-only

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

class ScanFilterProbe : public SlamToolbox
{
public:
  explicit ScanFilterProbe(const rclcpp::NodeOptions & options)
  : SlamToolbox(options) {}

  bool scan(int seconds, double x, double heading = 0.0)
  {
    auto message = std::make_shared<sensor_msgs::msg::LaserScan>();
    message->header.stamp.sec = seconds;
    return shouldProcessScan(message, karto::Pose2(x, 0.0, heading));
  }

  double minimumDistance()
  {
    return smapper_->getMapper()->getParamMinimumTravelDistance();
  }

protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr) override {}
};

class ScanFilterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void TearDown() override
  {
    for (auto & node : nodes_) {
      if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        node->cleanup();
      }
    }
    nodes_.clear();
  }

  std::shared_ptr<ScanFilterProbe> makeNode(
    double distance = 0.5, double heading = 0.5, bool precise = true,
    double interval = 0.0, int throttle = 1)
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=scan_filter_" + std::to_string(nodes_.size())});
    options.parameter_overrides({
      rclcpp::Parameter("use_map_saver", false),
      rclcpp::Parameter("enable_interactive_mode", false),
      rclcpp::Parameter("minimum_travel_distance", distance),
      rclcpp::Parameter("minimum_travel_heading", heading),
      rclcpp::Parameter("minimum_time_interval", interval),
      rclcpp::Parameter("throttle_scans", throttle),
      rclcpp::Parameter("check_min_dist_and_heading_precisely", precise)});
    auto node = std::make_shared<ScanFilterProbe>(options);
    nodes_.push_back(node);
    if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      throw std::runtime_error("Failed to configure scan filter test node");
    }
    return node;
  }

  void warmup(const std::shared_ptr<ScanFilterProbe> & node, int start, double x = 0.0)
  {
    ASSERT_TRUE(node->scan(start, x));
    for (int offset = 1; offset <= 3; ++offset) {
      EXPECT_FALSE(node->scan(start + offset, x));
    }
  }

  std::vector<std::shared_ptr<ScanFilterProbe>> nodes_;
};

TEST_F(ScanFilterTest, FreshConfigurationPreservesMotionFiltering)
{
  auto node = makeNode();
  warmup(node, 1);
  EXPECT_FALSE(node->scan(5, 0.2));
  EXPECT_TRUE(node->scan(6, 0.6));
  EXPECT_FALSE(node->scan(7, 0.6, 0.2));
  EXPECT_TRUE(node->scan(8, 0.6, 0.6));
}

TEST_F(ScanFilterTest, ReconfigurationRefreshesDistanceThreshold)
{
  for (bool precise : {false, true}) {
    SCOPED_TRACE(precise ? "precise filtering" : "default filtering");
    auto node = makeNode(1.0, 0.5, precise);
    warmup(node, 1);
    ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("minimum_travel_distance", 0.1)).successful);
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    warmup(node, 10);
    EXPECT_DOUBLE_EQ(node->minimumDistance(), 0.1);
    EXPECT_TRUE(node->scan(14, 0.2));
  }
}

TEST_F(ScanFilterTest, ReconfigurationRefreshesHeadingThreshold)
{
  auto node = makeNode(1.0, 1.0);
  warmup(node, 1);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("minimum_travel_heading", 0.1)).successful);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  warmup(node, 10);
  EXPECT_TRUE(node->scan(14, 0.0, 0.2));
}

TEST_F(ScanFilterTest, InstancesUseTheirOwnMotionThresholds)
{
  auto first = makeNode(1.0, 1.0);
  auto second = makeNode(0.1, 0.1);
  warmup(first, 1);
  EXPECT_FALSE(first->scan(5, 0.2));
  EXPECT_FALSE(first->scan(6, 0.0, 0.2));
  warmup(second, 10);
  EXPECT_TRUE(second->scan(14, 0.2));
  EXPECT_TRUE(second->scan(15, 0.2, 0.2));
}

TEST_F(ScanFilterTest, InstancesUseTheirOwnPreviousPose)
{
  auto first = makeNode();
  auto second = makeNode();
  warmup(first, 1);
  warmup(second, 10, 2.0);
  EXPECT_TRUE(first->scan(14, 2.0));
}

TEST_F(ScanFilterTest, InstancesUseTheirOwnPreviousTimestamp)
{
  auto first = makeNode(0.5, 0.5, true, 5.0);
  auto second = makeNode(0.5, 0.5, true, 5.0);
  warmup(first, 10);
  warmup(second, 100);
  EXPECT_TRUE(first->scan(20, 2.0));
}

TEST_F(ScanFilterTest, InstancesHaveIndependentThrottleCounters)
{
  auto first = makeNode(0.5, 0.5, true, 0.0, 2);
  auto second = makeNode(0.5, 0.5, true, 0.0, 2);
  warmup(first, 1);
  warmup(second, 10);
  EXPECT_FALSE(first->scan(5, 2.0));
  EXPECT_FALSE(second->scan(14, 2.0));
  EXPECT_TRUE(first->scan(6, 2.0));
  EXPECT_TRUE(second->scan(15, 2.0));
}

TEST_F(ScanFilterTest, ReconfigurationRestartsStartupFiltering)
{
  auto node = makeNode();
  warmup(node, 1);
  ASSERT_TRUE(node->scan(5, 2.0));
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(node->scan(10, 0.0));
  for (int seconds = 11; seconds <= 13; ++seconds) {
    EXPECT_FALSE(node->scan(seconds, 2.0));
  }
  EXPECT_TRUE(node->scan(14, 2.0));
}

}  // namespace slam_toolbox
