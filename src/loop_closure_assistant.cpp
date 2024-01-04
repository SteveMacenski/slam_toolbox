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

#include <unordered_map>
#include <memory>

#include "slam_toolbox/loop_closure_assistant.hpp"

namespace loop_closure_assistant
{

/*****************************************************************************/
template<class NodeT>
LoopClosureAssistant::LoopClosureAssistant(
  NodeT node,
  karto::Mapper * mapper,
  laser_utils::ScanHolder * scan_holder,
  PausedState & state, ProcessType & processor_type)
: mapper_(mapper), scan_holder_(scan_holder),
  interactive_mode_(false), state_(state),
  processor_type_(processor_type),
  clock_(node->get_clock()), logger_(node->get_logger()),
  parameters_interface_(node->get_node_parameters_interface())
/*****************************************************************************/
{
  if (!node->has_parameter("paused_processing")) {
    node->declare_parameter("paused_processing", false);
  }
  node->set_parameter(rclcpp::Parameter("paused_processing", false));
  if (!node->has_parameter("interactive_mode")) {
    node->declare_parameter("interactive_mode", false);
  }
  node->set_parameter(rclcpp::Parameter("interactive_mode", false));
  node->get_parameter("enable_interactive_mode", enable_interactive_mode_);

  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  solver_ = mapper_->getScanSolver();

  ssClear_manual_ = node->template create_service<slam_toolbox::srv::Clear>(
    "slam_toolbox/clear_changes", std::bind(&LoopClosureAssistant::clearChangesCallback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  ssLoopClosure_ = node->template create_service<slam_toolbox::srv::LoopClosure>(
    "slam_toolbox/manual_loop_closure", std::bind(&LoopClosureAssistant::manualLoopClosureCallback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  scan_publisher_ = node->template create_publisher<sensor_msgs::msg::LaserScan>(
    "slam_toolbox/scan_visualization", 10);
  interactive_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "slam_toolbox",
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_topics_interface(),
    node->get_node_services_interface());
  ssInteractive_ = node->template create_service<slam_toolbox::srv::ToggleInteractive>(
    "slam_toolbox/toggle_interactive_mode", std::bind(&LoopClosureAssistant::interactiveModeCallback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


  marker_publisher_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
    "slam_toolbox/graph_visualization", rclcpp::QoS(1));
  map_frame_ = node->get_parameter("map_frame").as_string();
}

/*****************************************************************************/
void LoopClosureAssistant::setMapper(karto::Mapper * mapper)
/*****************************************************************************/
{
  mapper_ = mapper;
}

/*****************************************************************************/
void LoopClosureAssistant::processInteractiveFeedback(const
  visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS)
  {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000,
      "Interactive mode is invalid outside processing mode.");
    return;
  }

  const int id = std::stoi(feedback->marker_name, nullptr, 10);

  // was depressed, something moved, and now released
  if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP &&
      feedback->mouse_point_valid)
  {
    addMovedNodes(id, Eigen::Vector3d(feedback->mouse_point.x,
      feedback->mouse_point.y, tf2::getYaw(feedback->pose.orientation)));
  }

  // is currently depressed, being moved before release
  if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    // get scan
    sensor_msgs::msg::LaserScan scan = scan_holder_->getCorrectedScan(id);

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
    geometry_msgs::msg::TransformStamped msg;
    tf2::convert(transform, msg.transform);
    msg.child_frame_id = "scan_visualization";
    msg.header.frame_id = feedback->header.frame_id;
    msg.header.stamp = clock_->now();
    tfB_->sendTransform(msg);

    scan.header.frame_id = "scan_visualization";
    scan.header.stamp = clock_->now();
    scan_publisher_->publish(scan);
  }
}

/*****************************************************************************/
void LoopClosureAssistant::publishGraph()
/*****************************************************************************/
{
  interactive_server_->clear();
  auto graph = solver_->getGraph();

  if (graph->size() == 0) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Graph size: %zu", graph->size());
  bool interactive_mode = false;
  {
    boost::mutex::scoped_lock lock(interactive_mutex_);
    interactive_mode = interactive_mode_;
  }

  const auto & vertices = mapper_->GetGraph()->GetVertices();
  const auto & edges = mapper_->GetGraph()->GetEdges();
  const auto & localization_vertices = mapper_->GetLocalizationVertices();

  int first_localization_id = std::numeric_limits<int>::max();
  if (!localization_vertices.empty()) {
    first_localization_id = localization_vertices.front().vertex->GetObject()->GetUniqueId();
  }

  visualization_msgs::msg::MarkerArray marray;

  // clear existing markers to account for any removed nodes
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = clock_->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marray.markers.push_back(clear);

  visualization_msgs::msg::Marker m = vis_utils::toMarker(map_frame_, "slam_toolbox", 0.1, clock_);

  // add map nodes
  for (const auto & sensor_name : vertices) {
    for (const auto & vertex : sensor_name.second) {
      m.color.g = vertex.first < first_localization_id ? 0.0 : 1.0;
      const auto & pose = vertex.second->GetObject()->GetCorrectedPose();
      m.id = vertex.first;
      m.pose.position.x = pose.GetX();
      m.pose.position.y = pose.GetY();

      if (interactive_mode && enable_interactive_mode_) {
        visualization_msgs::msg::InteractiveMarker int_marker =
          vis_utils::toInteractiveMarker(m, 0.3, clock_);
        interactive_server_->insert(int_marker,
          std::bind(
          &LoopClosureAssistant::processInteractiveFeedback,
          this, std::placeholders::_1));
      } else {
        marray.markers.push_back(m);
      }
    }
  }

  // add line markers for graph edges
  visualization_msgs::msg::Marker edges_marker;
  edges_marker.header.frame_id = map_frame_;
  edges_marker.header.stamp = clock_->now();
  edges_marker.id = 0;
  edges_marker.ns = "slam_toolbox_edges";
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.pose.orientation.w = 1;
  edges_marker.scale.x = 0.05;
  edges_marker.color.b = 1;
  edges_marker.color.a = 1;
  edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  edges_marker.points.reserve(edges.size() * 2);

  visualization_msgs::msg::Marker localization_edges_marker;
  localization_edges_marker.header.frame_id = map_frame_;
  localization_edges_marker.header.stamp = clock_->now();
  localization_edges_marker.id = 1;
  localization_edges_marker.ns = "slam_toolbox_edges";
  localization_edges_marker.action = visualization_msgs::msg::Marker::ADD;
  localization_edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  localization_edges_marker.pose.orientation.w = 1;
  localization_edges_marker.scale.x = 0.05;
  localization_edges_marker.color.g = 1;
  localization_edges_marker.color.b = 1;
  localization_edges_marker.color.a = 1;
  localization_edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  localization_edges_marker.points.reserve(localization_vertices.size() * 3);

  for (const auto & edge : edges) {
    int source_id = edge->GetSource()->GetObject()->GetUniqueId();
    const auto & pose0 = edge->GetSource()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p0;
    p0.x = pose0.GetX();
    p0.y = pose0.GetY();

    int target_id = edge->GetTarget()->GetObject()->GetUniqueId();
    const auto & pose1 = edge->GetTarget()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p1;
    p1.x = pose1.GetX();
    p1.y = pose1.GetY();

    if (source_id >= first_localization_id || target_id >= first_localization_id) {
      localization_edges_marker.points.push_back(p0);
      localization_edges_marker.points.push_back(p1);
    } else {
      edges_marker.points.push_back(p0);
      edges_marker.points.push_back(p1);
    }
  }

  marray.markers.push_back(edges_marker);
  marray.markers.push_back(localization_edges_marker);

  // if disabled, clears out old markers
  interactive_server_->applyChanges();
  marker_publisher_->publish(marray);
}

/*****************************************************************************/
bool LoopClosureAssistant::manualLoopClosureCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::LoopClosure::Request> req,
  std::shared_ptr<slam_toolbox::srv::LoopClosure::Response> resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    RCLCPP_WARN(
      logger_, "Called manual loop closure"
      " with interactive mode disabled. Ignoring.");
    return false;
  }

  {
    boost::mutex::scoped_lock lock(moved_nodes_mutex_);

    if (moved_nodes_.size() == 0)
    {
      RCLCPP_WARN(logger_, "No moved nodes to attempt manual loop closure.");
      return true;
    }

    RCLCPP_INFO(
      logger_,
      "LoopClosureAssistant: Attempting to manual "
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

  //update visualization and clear out nodes completed
  publishGraph();
  clearMovedNodes();
  return true;
}


/*****************************************************************************/
bool LoopClosureAssistant::interactiveModeCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::ToggleInteractive::Request>  req,
  std::shared_ptr<slam_toolbox::srv::ToggleInteractive::Response> resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    RCLCPP_WARN(
      logger_,
      "Called toggle interactive mode with interactive mode disabled. Ignoring.");
    return false;
  }

  bool interactive_mode;
  {
    boost::mutex::scoped_lock lock_i(interactive_mutex_);
    interactive_mode_ = !interactive_mode_;
    interactive_mode = interactive_mode_;
    parameters_interface_->set_parameters(
      {rclcpp::Parameter("interactive_mode", interactive_mode_)});
  }

  RCLCPP_INFO(
    logger_,
    "SlamToolbox: Toggling %s interactive mode.",
    interactive_mode ? "on" : "off");
  publishGraph();
  clearMovedNodes();

  // set state so we don't overwrite changes in rviz while loop closing
  state_.set(PROCESSING, interactive_mode);
  state_.set(VISUALIZING_GRAPH, interactive_mode);
  parameters_interface_->set_parameters(
    {rclcpp::Parameter("paused_processing", interactive_mode)});
  return true;
}

/*****************************************************************************/
void LoopClosureAssistant::moveNode(
  const int & id, const Eigen::Vector3d & pose)
/*****************************************************************************/
{
  solver_->ModifyNode(id, pose);
}

/*****************************************************************************/
bool LoopClosureAssistant::clearChangesCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::Clear::Request> req,
  std::shared_ptr<slam_toolbox::srv::Clear::Response> resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    RCLCPP_WARN(
      logger_,
      "Called Clear changes with interactive mode disabled. Ignoring.");
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "LoopClosureAssistant: Clearing manual loop closure nodes.");
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
void LoopClosureAssistant::addMovedNodes(const int & id, Eigen::Vector3d vec)
/*****************************************************************************/
{
  RCLCPP_INFO(
    logger_,
    "LoopClosureAssistant: Node %i new manual loop closure "
    "pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_[id] = vec;
}

// explicit instantiation for the supported template types
template LoopClosureAssistant::LoopClosureAssistant(
  rclcpp::Node::SharedPtr, karto::Mapper *, laser_utils::ScanHolder *, PausedState &,
  ProcessType &);
template LoopClosureAssistant::LoopClosureAssistant(
  rclcpp_lifecycle::LifecycleNode::SharedPtr, karto::Mapper *, laser_utils::ScanHolder *,
  PausedState &, ProcessType &);

}  // namespace loop_closure_assistant
