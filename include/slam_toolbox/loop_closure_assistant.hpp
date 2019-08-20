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

namespace loop_closure_assistant
{

class LoopClosureAssistant
{
public:
  LoopClosureAssistant(ros::NodeHandle& node, karto::Mapper* mapper, laser_utils::ScanHolder* scan_holder, std::function<void(void)> publishGraph);

  void clearMovedNodes();
  void processInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:
  bool manualLoopClosureCallback(slam_toolbox::LoopClosure::Request  &req, slam_toolbox::LoopClosure::Response &resp);
  bool clearChangesCallback(slam_toolbox::Clear::Request  &req, slam_toolbox::Clear::Response &resp);
  void moveNode(const int& id, const Eigen::Vector3d& pose);
  void addMovedNodes(const int& id, Eigen::Vector3d vec);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  std::unique_ptr<laser_utils::ScanHolder> scan_holder_;
  ros::Publisher scan_publisher_;
  ros::ServiceServer ssClear_manual_, ssLoopClosure_;
  boost::mutex moved_nodes_mutex_;
  std::map<int, Eigen::Vector3d> moved_nodes_;
  karto::Mapper* mapper_;
  karto::ScanSolver* solver_;
  std::function<void(void)> pubGraph_;
};

}  // end namespace

#endif //SLAM_TOOLBOX_LOOP_CLOSURE_ASSISTANT_H_
