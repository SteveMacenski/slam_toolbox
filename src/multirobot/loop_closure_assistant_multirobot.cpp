/*
 * loop_closure_assistant
 * Copyright (c) 2019, Samsung Research America
 * Copyright Work Modifications (c) 2024, Daniel I. Meza
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

#include "slam_toolbox/multirobot/loop_closure_assistant_multirobot.hpp"

namespace loop_closure_assistant
{

/*****************************************************************************/
LoopClosureAssistant::LoopClosureAssistant(
  rclcpp::Node::SharedPtr node,
  karto::Mapper * mapper,
  laser_utils::ScanHolder * scan_holder,
  PausedState & state, ProcessType & processor_type)
: mapper_(mapper), scan_holder_(scan_holder),
  interactive_mode_(false), node_(node), state_(state),
  processor_type_(processor_type)
/*****************************************************************************/
{
  node_->declare_parameter("paused_processing", false);
  node_->set_parameter(rclcpp::Parameter("paused_processing", false));
  node_->declare_parameter("interactive_mode", false);
  node_->set_parameter(rclcpp::Parameter("interactive_mode", false));
  node_->get_parameter("enable_interactive_mode", enable_interactive_mode_);

  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  solver_ = mapper_->getScanSolver();

  ssClear_manual_ = node_->create_service<slam_toolbox::srv::Clear>(
    "slam_toolbox/clear_changes", std::bind(&LoopClosureAssistant::clearChangesCallback, 
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  
  ssLoopClosure_ = node_->create_service<slam_toolbox::srv::LoopClosure>(
    "slam_toolbox/manual_loop_closure", std::bind(&LoopClosureAssistant::manualLoopClosureCallback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  
  scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "slam_toolbox/scan_visualization",10);
  interactive_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "slam_toolbox",
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_services_interface());
  ssInteractive_ = node_->create_service<slam_toolbox::srv::ToggleInteractive>(
    "slam_toolbox/toggle_interactive_mode", std::bind(&LoopClosureAssistant::interactiveModeCallback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
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
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5, 
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
    msg.header.stamp = node_->now();
    tfB_->sendTransform(msg);

    scan.header.frame_id = "scan_visualization";
    scan.header.stamp = node_->now();
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

  RCLCPP_DEBUG(node_->get_logger(), "Graph size: %zu", graph->size());
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

  std::map<karto::Name, visualization_msgs::msg::MarkerArray> m_sensor_name_to_marray;

  // Initialize marker array for each robot
  for (const auto & sensor_name : vertices) {
    m_sensor_name_to_marray[sensor_name.first] = visualization_msgs::msg::MarkerArray();
  }

  std::map<karto::Name, visualization_msgs::msg::Marker> m_sensor_name_to_vertex_marker;

  // Initialize vertex markers
  visualization_msgs::msg::Marker vertex_marker;
  vertex_marker.header.frame_id = map_frame_;
  vertex_marker.header.stamp = node_->now();
  vertex_marker.id = 0;
  // vertex_marker.ns = "slam_toolbox_";
  vertex_marker.action = visualization_msgs::msg::Marker::ADD;
  vertex_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.1;
  vertex_marker.scale.y = 0.1;
  vertex_marker.scale.z = 0.1;
  vertex_marker.color.a = 1;
  vertex_marker.lifetime = rclcpp::Duration::from_seconds(0);
  vertex_marker.points.reserve(vertices.size());

  // Initialize vertex marker for each sensor name in the map
  for (const auto & [sensor_name, nodes] : vertices) {
    // Get random color for new robots
    std::map<karto::Name, std_msgs::msg::ColorRGBA>::const_iterator map_it = m_sensor_name_to_color_.find(sensor_name);
    if (map_it == m_sensor_name_to_color_.end()) {
      m_sensor_name_to_color_[sensor_name] = generateNewColor();
    }

    // Parameters for each robot
    vertex_marker.ns = "vertices" + sensor_name.ToString();
    vertex_marker.color = m_sensor_name_to_color_[sensor_name];

    m_sensor_name_to_vertex_marker[sensor_name] = vertex_marker;

    // add map nodes
    for (const auto & vertex : nodes) {
      // m.color.g = vertex.first < first_localization_id ? 0.0 : 1.0;
      const auto & pose = vertex.second->GetObject()->GetCorrectedPose();
      geometry_msgs::msg::Point p;
      p.x = pose.GetX();
      p.y = pose.GetY();
      
      if (interactive_mode && enable_interactive_mode_) {
        visualization_msgs::msg::InteractiveMarker int_marker =
          vis_utils::toInteractiveMarker(vertex_marker, 0.3, node_);
        interactive_server_->insert(int_marker,
          std::bind(
          &LoopClosureAssistant::processInteractiveFeedback,
          this, std::placeholders::_1));
      } else {
        m_sensor_name_to_vertex_marker[sensor_name].points.push_back(p);
      }
    }
  }

  std::map<karto::Name, visualization_msgs::msg::Marker> m_sensor_name_to_edge_marker;
  std::map<karto::Name, visualization_msgs::msg::Marker> m_sensor_name_to_localization_edge_marker;

  // Initialize edge markers for connections between nodes
  // Edge marker
  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = map_frame_;
  edge_marker.header.stamp = node_->now();
  edge_marker.id = 0;
  // edge_marker.ns = "slam_toolbox_edges";
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.pose.orientation.w = 1;
  edge_marker.scale.x = 0.05;
  edge_marker.color.b = 1;
  edge_marker.color.a = 1;
  edge_marker.lifetime = rclcpp::Duration::from_seconds(0);
  edge_marker.points.reserve(edges.size());

  // Localization edge marker
  visualization_msgs::msg::Marker localization_edge_marker;
  localization_edge_marker.header.frame_id = map_frame_;
  localization_edge_marker.header.stamp = node_->now();
  localization_edge_marker.id = 1;
  // localization_edge_marker.ns = "slam_toolbox_edges";
  localization_edge_marker.action = visualization_msgs::msg::Marker::ADD;
  localization_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  localization_edge_marker.pose.orientation.w = 1;
  localization_edge_marker.scale.x = 0.05;
  localization_edge_marker.color.g = 1;
  localization_edge_marker.color.b = 1;
  localization_edge_marker.color.a = 1;
  localization_edge_marker.lifetime = rclcpp::Duration::from_seconds(0);
  localization_edge_marker.points.reserve(localization_vertices.size());

  // Initialize edge marker for inter robot connections
  auto inter_robot_markers_name = karto::Name("inter_robot_edges");

  visualization_msgs::msg::Marker inter_robot_edge_marker = edge_marker;
  inter_robot_edge_marker.ns = inter_robot_markers_name.ToString();

  visualization_msgs::msg::Marker inter_robot_localization_edge_marker = localization_edge_marker;
  inter_robot_localization_edge_marker.ns = inter_robot_markers_name.ToString();

  // Initialize edge marker for each sensor name in the map
  for (const auto & [sensor_name, marray] : m_sensor_name_to_marray) {
      edge_marker.ns = "edges" + sensor_name.ToString();
      edge_marker.color = m_sensor_name_to_color_[sensor_name];
      edge_marker.color.a = 0.7;

      localization_edge_marker.ns = "edges" + sensor_name.ToString();
      localization_edge_marker.color = m_sensor_name_to_color_[sensor_name];
      localization_edge_marker.color.a = 0.7;

    m_sensor_name_to_edge_marker[sensor_name] = edge_marker;
    m_sensor_name_to_localization_edge_marker[sensor_name] = localization_edge_marker;
  }

  // add node edges
  for (const auto & edge : edges) {
    int source_id = edge->GetSource()->GetObject()->GetUniqueId();
    karto::Name source_sensor_name = edge->GetSource()->GetObject()->GetSensorName();
    const auto & pose0 = edge->GetSource()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p0;
    p0.x = pose0.GetX();
    p0.y = pose0.GetY();

    int target_id = edge->GetTarget()->GetObject()->GetUniqueId();
    karto::Name target_sensor_name = edge->GetTarget()->GetObject()->GetSensorName();
    const auto & pose1 = edge->GetTarget()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p1;
    p1.x = pose1.GetX();
    p1.y = pose1.GetY();

    if (source_id >= first_localization_id || target_id >= first_localization_id) {
      if (source_sensor_name == target_sensor_name) {
        m_sensor_name_to_localization_edge_marker[source_sensor_name].points.push_back(p0);
        m_sensor_name_to_localization_edge_marker[source_sensor_name].points.push_back(p1);
      }
      else {
        inter_robot_localization_edge_marker.points.push_back(p0);
        inter_robot_edge_marker.points.push_back(p1);
      }
    } else {
      if (source_sensor_name.ToString() == target_sensor_name.ToString()) {
        m_sensor_name_to_edge_marker[source_sensor_name].points.push_back(p0);
        m_sensor_name_to_edge_marker[source_sensor_name].points.push_back(p1);
      }
      else {
        inter_robot_edge_marker.points.push_back(p0);
        inter_robot_edge_marker.points.push_back(p1);
      }
    }
  }

  // if disabled, clears out old markers
  interactive_server_->applyChanges();

  // Push markers for each robot and their connections
  for (const auto & [sensor_name, marray] : m_sensor_name_to_marray) {
    m_sensor_name_to_marray[sensor_name].markers.push_back(m_sensor_name_to_vertex_marker[sensor_name]);
    m_sensor_name_to_marray[sensor_name].markers.push_back(m_sensor_name_to_edge_marker[sensor_name]);
    m_sensor_name_to_marray[sensor_name].markers.push_back(m_sensor_name_to_localization_edge_marker[sensor_name]);
    marker_publisher_->publish(marray);
  }

  // Push markers for inter robot connections
  visualization_msgs::msg::MarkerArray marray;
  marray.markers.push_back(inter_robot_edge_marker);
  marray.markers.push_back(inter_robot_localization_edge_marker);
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
      node_->get_logger(), "Called manual loop closure"
      " with interactive mode disabled. Ignoring.");
    return false;
  }

  {
    boost::mutex::scoped_lock lock(moved_nodes_mutex_);

    if (moved_nodes_.size() == 0)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "No moved nodes to attempt manual loop closure.");
      return true;
    }

    RCLCPP_INFO(
      node_->get_logger(),
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
      node_->get_logger(),
      "Called toggle interactive mode with interactive mode disabled. Ignoring.");
    return false;
  }

  bool interactive_mode;
  {
    boost::mutex::scoped_lock lock_i(interactive_mutex_);
    interactive_mode_ = !interactive_mode_;
    interactive_mode = interactive_mode_;
    node_->set_parameter(rclcpp::Parameter("interactive_mode", interactive_mode_));
  }

  RCLCPP_INFO(node_->get_logger(),
     "SlamToolbox: Toggling %s interactive mode.",
      interactive_mode ? "on" : "off");
  publishGraph();
  clearMovedNodes();

  // set state so we don't overwrite changes in rviz while loop closing
  state_.set(PROCESSING, interactive_mode);
  state_.set(VISUALIZING_GRAPH, interactive_mode);
  node_->set_parameter(rclcpp::Parameter("paused_processing", interactive_mode));
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
      node_->get_logger(),
      "Called Clear changes with interactive mode disabled. Ignoring.");
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
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
    node_->get_logger(),
    "LoopClosureAssistant: Node %i new manual loop closure "
    "pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_[id] = vec;
}

/*****************************************************************************/
std_msgs::msg::ColorRGBA LoopClosureAssistant::generateNewColor()
/*****************************************************************************/
{
  // Generate random color
  std_msgs::msg::ColorRGBA color;     // ranges are from 0.0 - 1.0
  color.r = (float)(std::abs(rand()) % 256) / 256;
  color.g = (float)(std::abs(rand()) % 256) / 256;
  color.b = (float)(std::abs(rand()) % 256) / 256;
  color.a = 1;

  return color;
}

}  // namespace loop_closure_assistant
