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

#include "slam_toolbox/merge_maps_kinematic.hpp"
#include "slam_toolbox/serialization.hpp"

/*****************************************************************************/
MergeMapsKinematic::MergeMapsKinematic() : num_submaps_(0), nh_("map_merging")
/*****************************************************************************/
{
  ROS_INFO("MergeMapsKinematic: Starting up!");
  setup();
}

/*****************************************************************************/
void MergeMapsKinematic::setup()
/*****************************************************************************/
{
  nh_.param("resolution", resolution_, 0.05);
  sstS_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true));
  sstmS_.push_back(nh_.advertise<nav_msgs::MapMetaData>(
    "/map_metadata", 1, true));
  ssMap_ = nh_.advertiseService("merge_submaps",
    &MergeMapsKinematic::mergeMapCallback, this);
  ssSubmap_ = nh_.advertiseService("add_submap",
    &MergeMapsKinematic::addSubmapCallback, this);
  
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  
  interactive_server_ = 
    std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "merge_maps_tool","",true);
}

/*****************************************************************************/
MergeMapsKinematic::~MergeMapsKinematic()
/*****************************************************************************/
{
}

/*****************************************************************************/
bool MergeMapsKinematic::addSubmapCallback(
  slam_toolbox_msgs::AddSubmap::Request& req,
  slam_toolbox_msgs::AddSubmap::Response& resp)
/*****************************************************************************/
{
  std::unique_ptr<karto::Mapper> mapper = std::make_unique<karto::Mapper>();
  std::unique_ptr<karto::Dataset> dataset = std::make_unique<karto::Dataset>();

  if (!serialization::read(req.filename, *mapper, *dataset))
  {
    ROS_ERROR("addSubmapCallback: Failed to read "
      "file: %s.", req.filename.c_str());
    return true;
  }
  
  // we know the position because we put it there before any scans
  karto::LaserRangeFinder* laser = dynamic_cast<karto::LaserRangeFinder*>(
    dataset->GetLasers()[0]);
  dataset->Add(laser, true);
  dataset_vec_.push_back(std::move(dataset));

  if (lasers_.find(laser->GetName().GetName()) == lasers_.end())
  {
    laser_utils::LaserMetadata laserMeta(laser, false);
    lasers_[laser->GetName().GetName()] = laserMeta;
  }

  karto::LocalizedRangeScanVector scans = mapper->GetAllProcessedScans();
  scans_vec_.push_back(scans);
  num_submaps_++;

  // create and publish map with marker that will move the map around
  sstS_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>(
    "/map_"+std::to_string(num_submaps_), 1, true));
  sstmS_.push_back(nh_.advertise<nav_msgs::MapMetaData>(
    "/map_metadata_" + std::to_string(num_submaps_), 1, true));
  sleep(1.0);

  nav_msgs::GetMap::Response map;
  nav_msgs::OccupancyGrid& og = map.map; 
  try
  {
    kartoToROSOccupancyGrid(scans, map);
  } catch (const karto::Exception& e)
  {
    ROS_WARN("Failed to build grid to add submap, Exception: %s",
      e.GetErrorMessage().c_str());
    return false;
  }

  tf2::Transform transform;
  transform.setIdentity();
  transform.setOrigin(tf2::Vector3(og.info.origin.position.x +
    og.info.width * og.info.resolution / 2.0,
    og.info.origin.position.y + og.info.height * og.info.resolution / 2.0,
    0.));
  og.info.origin.position.x = - (og.info.width * og.info.resolution / 2.0);
  og.info.origin.position.y = - (og.info.height * og.info.resolution / 2.0);
  og.header.stamp = ros::Time::now();
  og.header.frame_id = "map_"+std::to_string(num_submaps_);
  sstS_[num_submaps_].publish(og);
  sstmS_[num_submaps_].publish(og.info);

  geometry_msgs::TransformStamped msg;
  tf2::convert(transform, msg.transform);
  msg.child_frame_id = "/map_"+std::to_string(num_submaps_);
  msg.header.frame_id = "/map";
  msg.header.stamp = ros::Time::now();
  tfB_->sendTransform(msg);

  submap_marker_transform_[num_submaps_] = 
    tf2::Transform(tf2::Quaternion(0.,0.,0.,1.0),
    tf2::Vector3(0,0,0)); //no initial correction -- identity mat
  submap_locations_[num_submaps_] = 
    Eigen::Vector3d(transform.getOrigin().getX(),
    transform.getOrigin().getY(),0.);

  // create an interactive marker for the base of this frame and attach it
  visualization_msgs::Marker m = 
    vis_utils::toVertexMarker("map", "merge_maps_tool", 2.0);
  m.pose.position.x =  transform.getOrigin().getX();
  m.pose.position.y =  transform.getOrigin().getY();
  m.id = num_submaps_;

  visualization_msgs::InteractiveMarker int_marker =
    vis_utils::toInteractiveMarker(m, 2.4);
  interactive_server_->insert(int_marker,
    boost::bind(&MergeMapsKinematic::processInteractiveFeedback, this, _1));
  interactive_server_->applyChanges();

  ROS_INFO("Map %s was loaded into topic %s!", req.filename.c_str(),
    ("/map_"+std::to_string(num_submaps_)).c_str());
  return true;
}

/*****************************************************************************/
karto::Pose2 MergeMapsKinematic::applyCorrection(const
  karto::Pose2& pose,
  const tf2::Transform& submap_correction)
/*****************************************************************************/
{
  tf2::Transform pose_tf, pose_corr;
  tf2::Quaternion q(0.,0.,0.,1.0);
  q.setRPY(0., 0., pose.GetHeading());
  pose_tf.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  pose_tf.setRotation(q);
  pose_corr = submap_correction * pose_tf;
  return karto::Pose2(pose_corr.getOrigin().x(), pose_corr.getOrigin().y(),
    tf2::getYaw(pose_corr.getRotation()));
}

/*****************************************************************************/
karto::Vector2<kt_double> MergeMapsKinematic::applyCorrection(const
  karto::Vector2<kt_double>&  pose,
  const tf2::Transform& submap_correction)
/*****************************************************************************/
{
  tf2::Transform pose_tf, pose_corr;
  pose_tf.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  pose_tf.setRotation(tf2::Quaternion(0.,0.,0.,1.0));
  pose_corr = submap_correction * pose_tf;
  return karto::Vector2<kt_double>(pose_corr.getOrigin().x(),
    pose_corr.getOrigin().y());
}

/*****************************************************************************/
void MergeMapsKinematic::transformScan(LocalizedRangeScansIt iter,
  tf2::Transform& submap_correction)
/*****************************************************************************/
{
  // TRANSFORM BARYCENTERR POSE
  const karto::Pose2 bary_center_pose = (*iter)->GetBarycenterPose();
  auto bary_center_pose_corr = 
    applyCorrection(bary_center_pose, submap_correction);
  (*iter)->SetBarycenterPose(bary_center_pose_corr);

  // TRANSFORM BOUNDING BOX POSITIONS
  karto::BoundingBox2 bbox = (*iter)->GetBoundingBox();
  const karto::Vector2<kt_double> bbox_min_corr = 
    applyCorrection(bbox.GetMinimum(), submap_correction);
  bbox.SetMinimum(bbox_min_corr);
  const karto::Vector2<kt_double> bbox_max_corr = 
    applyCorrection(bbox.GetMaximum(), submap_correction);
  bbox.SetMaximum(bbox_max_corr);
  (*iter)->SetBoundingBox(bbox);

  // TRANSFORM UNFILTERED POINTS USED
  karto::PointVectorDouble UPR_vec = (*iter)->GetPointReadings();
  for(karto::PointVectorDouble::iterator it_upr = UPR_vec.begin();
    it_upr != UPR_vec.end(); ++it_upr)
  {
    const karto::Vector2<kt_double> upr_corr = applyCorrection(
      *it_upr, submap_correction);
    it_upr->SetX(upr_corr.GetX());
    it_upr->SetY(upr_corr.GetY());
  }
  (*iter)->SetPointReadings(UPR_vec);

  // TRANSFORM CORRECTED POSE
  const karto::Pose2 corrected_pose = (*iter)->GetCorrectedPose();
  karto::Pose2 karto_robot_pose_corr = applyCorrection(
    corrected_pose, submap_correction);
  (*iter)->SetCorrectedPose(karto_robot_pose_corr);
  kt_bool dirty = false;
  (*iter)->SetIsDirty(dirty);

  // TRANSFORM ODOM POSE
  karto::Pose2 odom_pose = (*iter)->GetOdometricPose();
  karto::Pose2 karto_robot_pose_odom = applyCorrection(
    odom_pose, submap_correction);
  (*iter)->SetOdometricPose(karto_robot_pose_odom);
}

/*****************************************************************************/
bool MergeMapsKinematic::mergeMapCallback(
  slam_toolbox_msgs::MergeMaps::Request& req,
  slam_toolbox_msgs::MergeMaps::Response& resp)
/*****************************************************************************/
{
  ROS_INFO("Merging maps!");

  // transform all the scans into the new global map coordinates 
  int id = 0;
  karto::LocalizedRangeScanVector transformed_scans;
  for(LocalizedRangeScansVecIt it_LRV = scans_vec_.begin();
    it_LRV != scans_vec_.end(); ++it_LRV)
  {
    id++;
    for (LocalizedRangeScansIt iter = it_LRV->begin();
      iter != it_LRV->end(); ++iter)
    {
      tf2::Transform submap_correction = submap_marker_transform_[id];
      transformScan(iter, submap_correction);
      transformed_scans.push_back((*iter));
    }
  }

  // create the map
  nav_msgs::GetMap::Response map;
  try
  {
    kartoToROSOccupancyGrid(transformed_scans, map);
  } catch (const karto::Exception& e)
  {
    ROS_WARN("Failed to build grid to merge maps together, Exception: %s",
      e.GetErrorMessage().c_str());
  }

  // publish
  map.map.header.stamp = ros::Time::now();
  map.map.header.frame_id = "map";
  sstS_[0].publish(map.map);
  sstmS_[0].publish(map.map.info);
  return true;
}

/*****************************************************************************/
void MergeMapsKinematic::kartoToROSOccupancyGrid(
  const karto::LocalizedRangeScanVector& scans,
  nav_msgs::GetMap::Response& map)
/*****************************************************************************/
{
  karto::OccupancyGrid* occ_grid = NULL;
  occ_grid = karto::OccupancyGrid::CreateFromScans(scans, resolution_);
  if (!occ_grid)
  {
    ROS_INFO("MergeMapsKinematic: Could not make Karto occupancy grid.");
  }
  else
  {
    map.map.info.resolution = resolution_;
    vis_utils::toNavMap(occ_grid, map.map);
  }

  delete occ_grid;
  return;
}

/*****************************************************************************/
void MergeMapsKinematic::processInteractiveFeedback(const
  visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
/*****************************************************************************/
{
  const int id = std::stoi(feedback->marker_name,nullptr,10);

  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP && 
      feedback->mouse_point_valid)
  {
    tfScalar yaw = tf2::getYaw(feedback->pose.orientation);
    tf2::Quaternion quat(0.,0.,0.,1.0);
    tf2::convert(feedback->pose.orientation, quat); // relative

    tf2::Transform previous_submap_correction;
    previous_submap_correction.setIdentity();
    previous_submap_correction.setOrigin(tf2::Vector3(submap_locations_[id](0),
      submap_locations_[id](1), 0.));

    // update internal knowledge of submap locations
    submap_locations_[id] = Eigen::Vector3d(feedback->pose.position.x,
      feedback->pose.position.y,
      submap_locations_[id](2) + yaw);

    // add the map_N frame there
    tf2::Transform new_submap_location;
    new_submap_location.setOrigin(tf2::Vector3(submap_locations_[id](0),
      submap_locations_[id](1), 0.));
    new_submap_location.setRotation(quat);

    geometry_msgs::TransformStamped msg;
    tf2::convert(new_submap_location, msg.transform);
    msg.child_frame_id = "/map_"+std::to_string(id);
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    tfB_->sendTransform(msg);

    submap_marker_transform_[id] = submap_marker_transform_[id] *
      previous_submap_correction.inverse() * new_submap_location;
  }

  if (feedback->event_type ==
    visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    tfScalar yaw = tf2::getYaw(feedback->pose.orientation);
    tf2::Quaternion quat(0.,0.,0.,1.0);
    tf2::convert(feedback->pose.orientation, quat); // relative

    // add the map_N frame there
    tf2::Transform new_submap_location;
    new_submap_location.setOrigin(tf2::Vector3(feedback->pose.position.x,
      feedback->pose.position.y, 0.));
    new_submap_location.setRotation(quat);

    geometry_msgs::TransformStamped msg;
    tf2::convert(new_submap_location, msg.transform);
    msg.child_frame_id = "/map_"+std::to_string(id);
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    tfB_->sendTransform(msg);
  }
}

/*****************************************************************************/
int main(int argc, char** argv)
/*****************************************************************************/
{
  ros::init(argc, argv, "merge_maps_kinematic");
  MergeMapsKinematic mmk;
  ros::spin();
  return 0;
}
