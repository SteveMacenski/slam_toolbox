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

#include "slam_toolbox/loop_closure_assistant.hpp"

namespace loop_closure_assistant
{

/*****************************************************************************/
LoopClosureAssistant::LoopClosureAssistant(
  ros::NodeHandle& node,
  karto::Mapper* mapper,
  laser_utils::ScanHolder* scan_holder,
  std::function<void(void)> publishGraph)
: mapper_(mapper), pubGraph_(publishGraph), scan_holder_(scan_holder)
/*****************************************************************************/
{
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  ssClear_manual_ = node.advertiseService("clear_changes", &LoopClosureAssistant::clearChangesCallback, this);
  ssLoopClosure_ = node.advertiseService("manual_loop_closure", &LoopClosureAssistant::manualLoopClosureCallback, this);
  scan_publisher_ = node.advertise<sensor_msgs::LaserScan>("karto_scan_visualization",10);
  solver_ = mapper_->getScanSolver();
}

/*****************************************************************************/
void LoopClosureAssistant::processInteractiveFeedback(const
  visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
/*****************************************************************************/
{
  const int id = std::stoi(feedback->marker_name, nullptr, 10) - 1;

  // was depressed, something moved, and now released
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP && 
      feedback->mouse_point_valid)
  {
    addMovedNodes(id, Eigen::Vector3d(feedback->mouse_point.x,
      feedback->mouse_point.y, tf2::getYaw(feedback->pose.orientation)));
  }

  // is currently depressed, being moved before release
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    // get scan
    sensor_msgs::LaserScan scan = scan_holder_->getCorrectedScan(id);

    // get correct orientation
    tf2::Quaternion quat(0.,0.,0.,1.0), msg_quat(0.,0.,0.,1.0);
    double node_yaw, first_node_yaw;
    solver_->GetNodeOrientation(id, node_yaw);
    solver_->GetNodeOrientation(0, first_node_yaw);
    tf2::Quaternion q1(0.,0.,0.,1.0);
    q1.setEuler(0., 0., node_yaw - 3.14159);
    tf2::Quaternion q2(0.,0.,0.,1.0);
    q2.setEuler(0., 0., 3.14159); 
    quat *= q1;
    quat *= q2;

    // interactive move
    tf2::convert(feedback->pose.orientation, msg_quat);
    quat *= msg_quat;
    quat.normalize();

    // create correct transform
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(feedback->pose.position.x,
      feedback->pose.position.y, 0.));
    transform.setRotation(quat);

    // publish the scan visualization with transform
    geometry_msgs::TransformStamped msg;
    tf2::convert(transform, msg.transform);
    msg.child_frame_id = "karto_scan_visualization";
    msg.header.frame_id = feedback->header.frame_id;
    msg.header.stamp = ros::Time::now();
    tfB_->sendTransform(msg);

    scan.header.frame_id = "karto_scan_visualization";
    scan.header.stamp = ros::Time::now();
    scan_publisher_.publish(scan);
  }
}

/*****************************************************************************/
bool LoopClosureAssistant::manualLoopClosureCallback(
  slam_toolbox::LoopClosure::Request& req,
  slam_toolbox::LoopClosure::Response& resp)
/*****************************************************************************/
{
  {
    boost::mutex::scoped_lock lock(moved_nodes_mutex_);

    if (moved_nodes_.size() == 0)
    {
      ROS_WARN("No moved nodes to attempt manual loop closure.");
      return true;
    }

    ROS_INFO("LoopClosureAssistant: Attempting to manual "
      "loop close with %i moved nodes.", (int)moved_nodes_.size());
    // for each in node map
    std::map<int, Eigen::Vector3d>::const_iterator it = moved_nodes_.begin();
    for (it; it != moved_nodes_.end(); ++it)
    {
      moveNode(it->first,
        Eigen::Vector3d(it->second(0),it->second(1), it->second(2)));
    }
  }

  // optimize
  mapper_->CorrectPoses();

  // update visualization and clear out nodes completed
  pubGraph_();  
  clearMovedNodes();
  return true;
}

/*****************************************************************************/
void LoopClosureAssistant::moveNode(
  const int& id, const Eigen::Vector3d& pose)
/*****************************************************************************/
{
  solver_->ModifyNode(id, pose);
}

/*****************************************************************************/
bool LoopClosureAssistant::clearChangesCallback(
  slam_toolbox::Clear::Request& req,
  slam_toolbox::Clear::Response& resp)
/*****************************************************************************/
{
  ROS_INFO("LoopClosureAssistant: Clearing manual loop closure nodes.");
  pubGraph_();
  clearMovedNodes();
  return true;
}

/*****************************************************************************/
void  LoopClosureAssistant::clearMovedNodes()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_.clear();
}

/*****************************************************************************/
void LoopClosureAssistant::addMovedNodes(const int& id, Eigen::Vector3d vec)
/*****************************************************************************/
{
  ROS_INFO("LoopClosureAssistant: Node %i new manual loop closure "
    "pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_[id] = vec;
}

} // end namespace