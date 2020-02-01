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
  PausedState& state, ProcessType & processor_type)
: mapper_(mapper), scan_holder_(scan_holder),
  interactive_mode_(false), nh_(node), state_(state),
  processor_type_(processor_type)
/*****************************************************************************/
{
  node.setParam("paused_processing", false);
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  ssClear_manual_ = node.advertiseService("clear_changes",
    &LoopClosureAssistant::clearChangesCallback, this);
  ssLoopClosure_ = node.advertiseService("manual_loop_closure",
    &LoopClosureAssistant::manualLoopClosureCallback, this);
  scan_publisher_ = node.advertise<sensor_msgs::LaserScan>(
    "karto_scan_visualization",10);
  solver_ = mapper_->getScanSolver();
  interactive_server_ =
    std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "slam_toolbox","",true);
  ssInteractive_ = node.advertiseService("toggle_interactive_mode",
    &LoopClosureAssistant::interactiveModeCallback,this);
  node.setParam("interactive_mode", interactive_mode_);
  marker_publisher_ = node.advertise<visualization_msgs::MarkerArray>(
    "karto_graph_visualization",1);
  node.param("map_frame", map_frame_, std::string("map"));
  node.param("enable_interactive_mode", enable_interactive_mode_, false);
}

/*****************************************************************************/
void LoopClosureAssistant::processInteractiveFeedback(const
  visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS)
  {
    ROS_ERROR_THROTTLE(5.,
      "Interactive mode is invalid outside processing mode.");
    return;
  }

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
void LoopClosureAssistant::publishGraph()
/*****************************************************************************/
{
  interactive_server_->clear();
  std::unordered_map<int, Eigen::Vector3d>* graph = solver_->getGraph();

  if (graph->size() == 0)
  {
    return;
  }

  ROS_DEBUG("Graph size: %i",(int)graph->size());
  bool interactive_mode = false;
  {
    boost::mutex::scoped_lock lock(interactive_mutex_);
    interactive_mode = interactive_mode_;
  }

  visualization_msgs::MarkerArray marray;
  visualization_msgs::Marker m = vis_utils::toMarker(map_frame_,
    "slam_toolbox", 0.1);

  for (ConstGraphIterator it = graph->begin(); it != graph->end(); ++it)
  {
    m.id = it->first + 1;
    m.pose.position.x = it->second(0);
    m.pose.position.y = it->second(1);

    if (interactive_mode && enable_interactive_mode_)
    {
      visualization_msgs::InteractiveMarker int_marker =
        vis_utils::toInteractiveMarker(m, 0.3);
      interactive_server_->insert(int_marker,
        boost::bind(
        &LoopClosureAssistant::processInteractiveFeedback,
        this, _1));
    }
    else
    {
      marray.markers.push_back(m);
    }
  }

  // if disabled, clears out old markers
  interactive_server_->applyChanges();
  marker_publisher_.publish(marray);
  return;
}

/*****************************************************************************/
bool LoopClosureAssistant::manualLoopClosureCallback(
  slam_toolbox_msgs::LoopClosure::Request& req,
  slam_toolbox_msgs::LoopClosure::Response& resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    ROS_WARN("Called manual loop closure"
      " with interactive mode disabled. Ignoring.");
    return false;
  }

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
  publishGraph();  
  clearMovedNodes();
  return true;
}

/*****************************************************************************/
bool LoopClosureAssistant::interactiveModeCallback(
  slam_toolbox_msgs::ToggleInteractive::Request  &req,
  slam_toolbox_msgs::ToggleInteractive::Response &resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    ROS_WARN("Called toggle interactive mode with "
      "interactive mode disabled. Ignoring.");
    return false;
  }

  bool interactive_mode;
  {
    boost::mutex::scoped_lock lock_i(interactive_mutex_);
    interactive_mode_ = !interactive_mode_;   
    interactive_mode = interactive_mode_;
    nh_.setParam("interactive_mode", interactive_mode_);
  }

  ROS_INFO("SlamToolbox: Toggling %s interactive mode.", 
    interactive_mode ? "on" : "off");
  publishGraph();
  clearMovedNodes();

  // set state so we don't overwrite changes in rviz while loop closing
  state_.set(PROCESSING, interactive_mode);
  state_.set(VISUALIZING_GRAPH, interactive_mode);
  nh_.setParam("paused_processing", interactive_mode);
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
  slam_toolbox_msgs::Clear::Request& req,
  slam_toolbox_msgs::Clear::Response& resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    ROS_WARN("Called Clear changes with interactive mode disabled. Ignoring.");
    return false;
  }

  ROS_INFO("LoopClosureAssistant: Clearing manual loop closure nodes.");
  publishGraph();
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