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

#include <boost/thread.hpp>
#include <string>
#include <functional>
#include <memory>
#include <map>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
// #include "interactive_markers/interactive_marker_server.h"
// #include "interactive_markers/menu_handler.h"

#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/laser_utils.hpp"
#include "slam_toolbox/visualization_utils.hpp"

namespace loop_closure_assistant
{

// TODO(stevemacenski): Need interactive markers ported to ROS2

using namespace ::toolbox_types;  // NOLINT

class LoopClosureAssistant
{
public:
  LoopClosureAssistant(
    rclcpp::Node::SharedPtr node, karto::Mapper * mapper,
    laser_utils::ScanHolder * scan_holder, PausedState & state,
    ProcessType & processor_type);

  // void clearMovedNodes();
  // void processInteractiveFeedback(
  //   const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void publishGraph();

private:
  // bool manualLoopClosureCallback(
  //   slam_toolbox::LoopClosure::Request& req, slam_toolbox::LoopClosure::Response& resp);
  // bool clearChangesCallback(
  //   slam_toolbox::Clear::Request& req, slam_toolbox::Clear::Response& resp);
  // bool interactiveModeCallback(
  //   slam_toolbox::ToggleInteractive::Request  &req,
  //   slam_toolbox::ToggleInteractive::Response &resp);
  // void moveNode(const int& id, const Eigen::Vector3d& pose);
  // void addMovedNodes(const int& id, Eigen::Vector3d vec);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  laser_utils::ScanHolder * scan_holder_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  // rclcpp::Publisher::SharedPtr<sensor_msgs::msg::LaserScan> scan_publisher_;
  // ros::ServiceServer ssClear_manual_, ssLoopClosure_, ssInteractive_;
  // boost::mutex moved_nodes_mutex_;
  // std::map<int, Eigen::Vector3d> moved_nodes_;
  karto::Mapper * mapper_;
  karto::ScanSolver * solver_;
  // std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  // boost::mutex interactive_mutex_;
  // bool interactive_mode_, enable_interactive_mode_;
  rclcpp::Node::SharedPtr node_;
  std::string map_frame_;
  PausedState & state_;
  ProcessType & processor_type_;
};

}   // namespace loop_closure_assistant

#endif  // SLAM_TOOLBOX__LOOP_CLOSURE_ASSISTANT_HPP_
