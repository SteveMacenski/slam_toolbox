/*
 * loop_closure_assistant
 * Copyright (c) 2019, Samsung Research America
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

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX__LOOP_CLOSURE_ASSISTANT_HPP_
#define SLAM_TOOLBOX__LOOP_CLOSURE_ASSISTANT_HPP_

#include <thread>
#include <string>
#include <functional>
#include <memory>
#include <map>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"

#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/laser_utils.hpp"
#include "slam_toolbox/visualization_utils.hpp"

namespace loop_closure_assistant
{

using namespace ::toolbox_types;  // NOLINT

class LoopClosureAssistant
{
public:
  LoopClosureAssistant(
    rclcpp::Node::SharedPtr node, karto::Mapper * mapper,
    laser_utils::ScanHolder * scan_holder, PausedState & state,
    ProcessType & processor_type);

  void clearMovedNodes();
  void publishGraph();
  void setMapper(karto::Mapper * mapper);

private:

  void moveNode(const int& id, const Eigen::Vector3d& pose);
  void addMovedNodes(const int& id, Eigen::Vector3d vec);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  laser_utils::ScanHolder * scan_holder_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  boost::mutex moved_nodes_mutex_;
  std::map<int, Eigen::Vector3d> moved_nodes_;
  karto::Mapper * mapper_;
  karto::ScanSolver * solver_;
  rclcpp::Node::SharedPtr node_;
  std::string map_frame_;
  PausedState & state_;
  ProcessType & processor_type_;
};

}   // namespace loop_closure_assistant

#endif  // SLAM_TOOLBOX__LOOP_CLOSURE_ASSISTANT_HPP_
