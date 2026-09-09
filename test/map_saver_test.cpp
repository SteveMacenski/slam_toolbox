/*
 * slam_toolbox
 * Copyright (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "slam_toolbox/map_saver.hpp"

namespace map_saver
{

class TestableMapSaver : public MapSaver
{
public:
  using MapSaver::MapSaver;
  using MapSaver::saveMapCallback;
};

class MapSaverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("map_saver_test");
    saver_ = std::make_unique<TestableMapSaver>(node_, "map");
    // second subscription on the same topic, used to know when the map was delivered
    bool delivered = false;
    sentinel_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1),
      [&delivered](nav_msgs::msg::OccupancyGrid::SharedPtr) {delivered = true;});
    pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local());
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.05;
    grid.info.width = 2;
    grid.info.height = 2;
    grid.data = {0, 0, 100, -1};
    pub_->publish(grid);
    for (int i = 0; i < 250 && !delivered; ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    for (int i = 0; i < 10; ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(delivered);
  }

  static bool fileExists(const std::string & path)
  {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<TestableMapSaver> saver_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sentinel_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

TEST_F(MapSaverTest, RejectsShellMetacharactersInName)
{
  const std::string canary =
    "/tmp/slam_toolbox_map_saver_canary_" + std::to_string(getpid());
  std::remove(canary.c_str());

  auto req = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
  auto res = std::make_shared<slam_toolbox::srv::SaveMap::Response>();
  req->name.data = "x; touch " + canary + " #";
  saver_->saveMapCallback(nullptr, req, res);

  ASSERT_NE(res->result, res->RESULT_NO_MAP_RECEIEVD);
  EXPECT_FALSE(fileExists(canary)) << "command embedded in the map name was executed";
  EXPECT_EQ(res->result, res->RESULT_UNDEFINED_FAILURE);
  std::remove(canary.c_str());
}

}  // namespace map_saver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
