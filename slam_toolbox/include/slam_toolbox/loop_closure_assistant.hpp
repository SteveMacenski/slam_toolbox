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

#ifndef SLAM_TOOLBOX_LOOP_CLOSURE_ASSISTANT_H_
#define SLAM_TOOLBOX_LOOP_CLOSURE_ASSISTANT_H_

#include <functional>
#include <boost/thread.hpp>
#include <map>

#include "ros/ros.h"
#include "interactive_markers/interactive_marker_server.h"
#include "interactive_markers/menu_handler.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"

#include "karto_sdk/Mapper.h"
#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/laser_utils.hpp"
#include "slam_toolbox/visualization_utils.hpp"

namespace loop_closure_assistant
{

using namespace ::toolbox_types;

class LoopClosureAssistant
{
public:
  LoopClosureAssistant(ros::NodeHandle& node, karto::Mapper* mapper, laser_utils::ScanHolder* scan_holder, PausedState& state, ProcessType& processor_type);

  void clearMovedNodes();
  void processInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void publishGraph();
  void setMapper(karto::Mapper * mapper);

private:
  bool manualLoopClosureCallback(slam_toolbox_msgs::LoopClosure::Request& req, slam_toolbox_msgs::LoopClosure::Response& resp);
  bool clearChangesCallback(slam_toolbox_msgs::Clear::Request& req, slam_toolbox_msgs::Clear::Response& resp);
  bool interactiveModeCallback(slam_toolbox_msgs::ToggleInteractive::Request  &req, slam_toolbox_msgs::ToggleInteractive::Response &resp);
  void moveNode(const int& id, const Eigen::Vector3d& pose);
  void addMovedNodes(const int& id, Eigen::Vector3d vec);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  laser_utils::ScanHolder* scan_holder_;
  ros::Publisher scan_publisher_, marker_publisher_;
  ros::ServiceServer ssClear_manual_, ssLoopClosure_, ssInteractive_;
  boost::mutex moved_nodes_mutex_;
  std::map<int, Eigen::Vector3d> moved_nodes_;
  karto::Mapper* mapper_;
  karto::ScanSolver* solver_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  visualization_msgs::MarkerArray marker_array_;
  boost::mutex interactive_mutex_;
  bool interactive_mode_, enable_interactive_mode_;
  ros::NodeHandle& nh_;
  std::string map_frame_;
  PausedState& state_;
  ProcessType& processor_type_;
};

}  // end namespace

#endif //SLAM_TOOLBOX_LOOP_CLOSURE_ASSISTANT_H_
