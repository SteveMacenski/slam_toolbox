/*
 * Author
 * Copyright (c) 2018, Simbe Robotics, Inc.
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

#include <memory>
#include <utility>
#include "slam_toolbox/merge_maps_kinematic.hpp"
#include "slam_toolbox/serialization.hpp"

/*****************************************************************************/
MergeMapsKinematic::MergeMapsKinematic()
: Node("map_merging")
/*****************************************************************************/
{
  RCLCPP_INFO(get_logger(), "MergeMapsKinematic: Starting up!");
  num_submaps_ = 0;
}

/*****************************************************************************/
void MergeMapsKinematic::configure()
/*****************************************************************************/
{
  resolution_ = 0.05;
  min_pass_through_ = 2;
  occupancy_threshold_ = 0.1;
  resolution_ = this->declare_parameter("resolution", resolution_);
  min_pass_through_ = this->declare_parameter("min_pass_through", min_pass_through_);
  occupancy_threshold_ = this->declare_parameter("occupancy_threshold", occupancy_threshold_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

  sstS_.push_back(this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", qos));
  sstmS_.push_back(this->create_publisher<nav_msgs::msg::MapMetaData>(
      "/map_metadata", qos));

  ssMap_ = this->create_service<slam_toolbox::srv::MergeMaps>("slam_toolbox/merge_submaps",
      std::bind(&MergeMapsKinematic::mergeMapCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
  ssSubmap_ = this->create_service<slam_toolbox::srv::AddSubmap>("slam_toolbox/add_submap",
      std::bind(&MergeMapsKinematic::addSubmapCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

  interactive_server_ =
    std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "merge_maps_tool", shared_from_this());
}

/*****************************************************************************/
MergeMapsKinematic::~MergeMapsKinematic()
/*****************************************************************************/
{
}

/*****************************************************************************/
bool MergeMapsKinematic::addSubmapCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::AddSubmap::Request> req,
  std::shared_ptr<slam_toolbox::srv::AddSubmap::Response> resp)
/*****************************************************************************/
{
  std::unique_ptr<Mapper> mapper = std::make_unique<Mapper>();
  std::unique_ptr<Dataset> dataset = std::make_unique<Dataset>();

  if (!serialization::read(req->filename, *mapper,
    *dataset, shared_from_this()))
  {
    RCLCPP_ERROR(get_logger(), "addSubmapCallback: Failed to read "
      "file: %s.", req->filename.c_str());
    return true;
  }

  // we know the position because we put it there before any scans
  LaserRangeFinder * laser = dynamic_cast<LaserRangeFinder *>(
    dataset->GetLasers()[0]);
  dataset->Add(laser, true);
  dataset_vec_.push_back(std::move(dataset));

  if (lasers_.find(laser->GetName().GetName()) == lasers_.end()) {
    laser_utils::LaserMetadata laserMeta(laser, false);
    lasers_[laser->GetName().GetName()] = laserMeta;
  }

  LocalizedRangeScanVector scans = mapper->GetAllProcessedScans();
  scans_vec_.push_back(scans);
  num_submaps_++;

  // create and publish map with marker that will move the map around
  sstS_.push_back(this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map_" + std::to_string(num_submaps_), rclcpp::QoS(1)));
  sstmS_.push_back(this->create_publisher<nav_msgs::msg::MapMetaData>(
      "/map_metadata_" + std::to_string(num_submaps_), rclcpp::QoS(1)));
  sleep(1.0);

  nav_msgs::srv::GetMap::Response map;
  nav_msgs::msg::OccupancyGrid & og = map.map;
  try {
    kartoToROSOccupancyGrid(scans, map);
  } catch (const Exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to build grid to add submap, Exception: %s",
      e.GetErrorMessage().c_str());
    return false;
  }

  tf2::Transform transform;
  transform.setIdentity();
  transform.setOrigin(tf2::Vector3(og.info.origin.position.x +
    og.info.width * og.info.resolution / 2.0,
    og.info.origin.position.y + og.info.height * og.info.resolution / 2.0,
    0.));
  og.info.origin.position.x = -(og.info.width * og.info.resolution / 2.0);
  og.info.origin.position.y = -(og.info.height * og.info.resolution / 2.0);
  og.header.stamp = this->now();
  og.header.frame_id = "map_" + std::to_string(num_submaps_);
  sstS_[num_submaps_]->publish(og);
  sstmS_[num_submaps_]->publish(og.info);

  geometry_msgs::msg::TransformStamped msg;
  msg.transform = tf2::toMsg(transform);
  msg.child_frame_id = "/map_" + std::to_string(num_submaps_);
  msg.header.frame_id = "/map";
  msg.header.stamp = this->now();
  tfB_->sendTransform(msg);

  submap_marker_transform_[num_submaps_] =
    tf2::Transform(tf2::Quaternion(0., 0., 0., 1.0),
      tf2::Vector3(0, 0, 0));  // no initial correction -- identity mat
  prev_submap_marker_transform_ = submap_marker_transform_;
  submap_locations_[num_submaps_] =
    Eigen::Vector3d(transform.getOrigin().getX(),
      transform.getOrigin().getY(), 0.);

  // create an interactive marker for the base of this frame and attach it
  visualization_msgs::msg::Marker m =
    vis_utils::toMarker("map", "merge_maps_tool", 2.0, shared_from_this());
  m.pose.position.x = transform.getOrigin().getX();
  m.pose.position.y = transform.getOrigin().getY();
  m.id = num_submaps_;

  visualization_msgs::msg::InteractiveMarker int_marker =
    vis_utils::toInteractiveMarker(m, 2.4, shared_from_this());
  interactive_server_->insert(
    int_marker,
    std::bind(&MergeMapsKinematic::processInteractiveFeedback, this, std::placeholders::_1));
  interactive_server_->applyChanges();

  RCLCPP_INFO(get_logger(),
    "Map %s was loaded into topic %s!", req->filename.c_str(),
    ("/map_" + std::to_string(num_submaps_)).c_str());
  return true;
}

/*****************************************************************************/
Pose2 MergeMapsKinematic::applyCorrection(
  const
  Pose2 & pose,
  const tf2::Transform & submap_correction)
/*****************************************************************************/
{
  tf2::Transform pose_tf, pose_corr;
  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., pose.GetHeading());
  pose_tf.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  pose_tf.setRotation(q);
  pose_corr = submap_correction * pose_tf;
  return Pose2(pose_corr.getOrigin().x(), pose_corr.getOrigin().y(),
           tf2::getYaw(pose_corr.getRotation()));
}

/*****************************************************************************/
Vector2<kt_double> MergeMapsKinematic::applyCorrection(
  const
  Vector2<kt_double> & pose,
  const tf2::Transform & submap_correction)
/*****************************************************************************/
{
  tf2::Transform pose_tf, pose_corr;
  pose_tf.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  pose_tf.setRotation(tf2::Quaternion(0., 0., 0., 1.0));
  pose_corr = submap_correction * pose_tf;
  return Vector2<kt_double>(pose_corr.getOrigin().x(),
           pose_corr.getOrigin().y());
}

/*****************************************************************************/
void MergeMapsKinematic::transformScan(
  LocalizedRangeScansIt iter,
  tf2::Transform & submap_correction)
/*****************************************************************************/
{
  // TRANSFORM BARYCENTERR POSE
  const Pose2 bary_center_pose = (*iter)->GetBarycenterPose();
  auto bary_center_pose_corr =
    applyCorrection(bary_center_pose, submap_correction);
  (*iter)->SetBarycenterPose(bary_center_pose_corr);

  // TRANSFORM BOUNDING BOX POSITIONS
  BoundingBox2 bbox = (*iter)->GetBoundingBox();
  const Vector2<kt_double> bbox_min_corr =
    applyCorrection(bbox.GetMinimum(), submap_correction);
  const Vector2<kt_double> bbox_max_corr =
    applyCorrection(bbox.GetMaximum(), submap_correction);
  Vector2<kt_double> bbox_min_right_corr{bbox.GetMaximum().GetX(),bbox.GetMinimum().GetY()};
  bbox_min_right_corr = applyCorrection(bbox_min_right_corr, submap_correction);
  Vector2<kt_double> bbox_max_left_corr{bbox.GetMinimum().GetX(),bbox.GetMaximum().GetY()};
  bbox_max_left_corr = applyCorrection(bbox_max_left_corr, submap_correction);
  BoundingBox2 transformed_bbox;
  transformed_bbox.Add(bbox_min_corr);
  transformed_bbox.Add(bbox_max_corr);
  transformed_bbox.Add(bbox_min_right_corr);
  transformed_bbox.Add(bbox_max_left_corr);
  (*iter)->SetBoundingBox(transformed_bbox);

  // TRANSFORM UNFILTERED POINTS USED
  PointVectorDouble UPR_vec = (*iter)->GetPointReadings();
  for (PointVectorDouble::iterator it_upr = UPR_vec.begin();
    it_upr != UPR_vec.end(); ++it_upr)
  {
    const Vector2<kt_double> upr_corr = applyCorrection(
      *it_upr, submap_correction);
    it_upr->SetX(upr_corr.GetX());
    it_upr->SetY(upr_corr.GetY());
  }
  (*iter)->SetPointReadings(UPR_vec);

  // TRANSFORM CORRECTED POSE
  const Pose2 corrected_pose = (*iter)->GetCorrectedPose();
  Pose2 robot_pose_corr = applyCorrection(
    corrected_pose, submap_correction);
  (*iter)->SetCorrectedPose(robot_pose_corr);
  kt_bool dirty = false;
  (*iter)->SetIsDirty(dirty);

  // TRANSFORM ODOM POSE
  Pose2 odom_pose = (*iter)->GetOdometricPose();
  Pose2 robot_pose_odom = applyCorrection(
    odom_pose, submap_correction);
  (*iter)->SetOdometricPose(robot_pose_odom);
}

/*****************************************************************************/
bool MergeMapsKinematic::mergeMapCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::MergeMaps::Request> req,
  std::shared_ptr<slam_toolbox::srv::MergeMaps::Response> resp)
/*****************************************************************************/
{
  RCLCPP_INFO(get_logger(), "Merging maps!");

  // transform all the scans into the new global map coordinates
  int id = 0;
  LocalizedRangeScanVector transformed_scans;
  for (LocalizedRangeScansVecIt it_LRV = scans_vec_.begin();
    it_LRV != scans_vec_.end(); ++it_LRV)
  {
    id++;
    for (LocalizedRangeScansIt iter = it_LRV->begin();
      iter != it_LRV->end(); ++iter)
    {
      tf2::Transform submap_correction = submap_marker_transform_[id] * prev_submap_marker_transform_[id].inverse();
      transformScan(iter, submap_correction);
      transformed_scans.push_back((*iter));
    }
  }
  prev_submap_marker_transform_ = submap_marker_transform_;

  // create the map
  nav_msgs::srv::GetMap::Response map;
  try {
    kartoToROSOccupancyGrid(transformed_scans, map);
  } catch (const Exception & e) {
    RCLCPP_WARN(get_logger(),
      "Failed to build grid to merge maps together, Exception: %s",
      e.GetErrorMessage().c_str());
  }

  // publish
  map.map.header.stamp = this->now();
  map.map.header.frame_id = "map";
  sstS_[0]->publish(map.map);
  sstmS_[0]->publish(map.map.info);
  return true;
}

/*****************************************************************************/
void MergeMapsKinematic::kartoToROSOccupancyGrid(
  const LocalizedRangeScanVector & scans,
  nav_msgs::srv::GetMap::Response & map)
/*****************************************************************************/
{
  OccupancyGrid * occ_grid = NULL;
  occ_grid = OccupancyGrid::CreateFromScans(scans, resolution_, min_pass_through_, occupancy_threshold_);
  if (!occ_grid) {
    RCLCPP_INFO(get_logger(),
      "MergeMapsKinematic: Could not make occupancy grid.");
  } else {
    map.map.info.resolution = resolution_;
    vis_utils::toNavMap(occ_grid, map.map);
  }

  delete occ_grid;
}

/*****************************************************************************/
void MergeMapsKinematic::processInteractiveFeedback(
  visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
/*****************************************************************************/
{
  const int id = std::stoi(feedback->marker_name, nullptr, 10);

  tf2::Quaternion quat(feedback->pose.orientation.x,
    feedback->pose.orientation.y,
    feedback->pose.orientation.z,
    feedback->pose.orientation.w);
  
  if (feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP &&
    feedback->mouse_point_valid)
  {
    tf2Scalar yaw = tf2::getYaw(feedback->pose.orientation);
    tf2::fromMsg(feedback->pose.orientation, quat);  // relative

    tf2::Transform previous_submap_correction;
    previous_submap_correction.setIdentity();
    previous_submap_correction.setOrigin(tf2::Vector3(submap_locations_[id](0),
      submap_locations_[id](1), 0.));
    previous_submap_correction.setRotation(submap_marker_transform_[id].getRotation());
    // update internal knowledge of submap locations
    submap_locations_[id] = Eigen::Vector3d(feedback->pose.position.x,
        feedback->pose.position.y,
        0.0);

    // add the map_N frame there
    tf2::Transform new_submap_location;
    new_submap_location.setOrigin(tf2::Vector3(feedback->pose.position.x,
      feedback->pose.position.y, 0.));
    new_submap_location.setRotation(quat);
    geometry_msgs::msg::TransformStamped msg;
    msg.transform = tf2::toMsg(new_submap_location);
    msg.child_frame_id = "/map_" + std::to_string(id);
    msg.header.frame_id = "/map";
    msg.header.stamp = this->now();
    tfB_->sendTransform(msg);

    submap_marker_transform_[id] = new_submap_location;
  }

  if (feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    tf2Scalar yaw = tf2::getYaw(feedback->pose.orientation);
    tf2::fromMsg(feedback->pose.orientation, quat);  // relative

    // add the map_N frame there
    tf2::Transform new_submap_location;
    new_submap_location.setOrigin(tf2::Vector3(feedback->pose.position.x,
      feedback->pose.position.y, 0.));
    new_submap_location.setRotation(quat);

    geometry_msgs::msg::TransformStamped msg;
    msg.transform = tf2::toMsg(new_submap_location);
    msg.child_frame_id = "/map_" + std::to_string(id);
    msg.header.frame_id = "/map";
    msg.header.stamp = this->now();
    tfB_->sendTransform(msg);
  }
  this->mergeMapCallback(nullptr, nullptr, nullptr);
}

/*****************************************************************************/
int main(int argc, char ** argv)
/*****************************************************************************/
{
  rclcpp::init(argc, argv);
  auto merging_node = std::make_shared<MergeMapsKinematic>();
  merging_node->configure();
  rclcpp::spin(merging_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
