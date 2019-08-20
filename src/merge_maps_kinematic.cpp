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
  slam_toolbox::AddSubmap::Request &req,
  slam_toolbox::AddSubmap::Response &resp)
/*****************************************************************************/
{
  std::unique_ptr<karto::Mapper> mapper = std::make_unique<karto::Mapper>();
  std::unique_ptr<karto::Dataset> dataset = std::make_unique<karto::Dataset>();

  serialization::read(req.filename, *mapper, *dataset);

  // we know the position because we put it there before any scans
  karto::LaserRangeFinder* laser = dynamic_cast<karto::LaserRangeFinder*>(
    dataset->GetObjects()[0]);
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
  } catch (const karto::Exception & e)
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

  submap_marker_transform_[num_submaps_]=tf2::Transform(tf2::Quaternion(0.,0.,0.,1.0),
    tf2::Vector3(0,0,0)); //no initial correction -- identity mat
  submap_locations_[num_submaps_] = Eigen::Vector3d(transform.getOrigin().getX(),
    transform.getOrigin().getY(),0.);

  // create an interactive marker for the base of this frame and attach it
  visualization_msgs::Marker m = vis_utils::toMarker("map", "merge_maps_tool", 2.0);
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
  tf2::Transform poseTf, poseCorr;
  tf2::Quaternion q(0.,0.,0.,1.0);
  q.setRPY(0., 0., pose.GetHeading());
  poseTf.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  poseTf.setRotation(q);
  poseCorr = submap_correction * poseTf;
  return karto::Pose2(poseCorr.getOrigin().x(), poseCorr.getOrigin().y(),
    tf2::getYaw(poseCorr.getRotation()));
}

/*****************************************************************************/
karto::Vector2<kt_double> MergeMapsKinematic::applyCorrection(const
  karto::Vector2<kt_double>&  pose,
  const tf2::Transform& submap_correction)
/*****************************************************************************/
{
  tf2::Transform poseTf, poseCorr;
  poseTf.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  poseTf.setRotation(tf2::Quaternion(0.,0.,0.,1.0));
  poseCorr = submap_correction * poseTf;
  return karto::Vector2<kt_double>(poseCorr.getOrigin().x(), poseCorr.getOrigin().y());
}

/*****************************************************************************/
bool MergeMapsKinematic::mergeMapCallback(
  slam_toolbox::MergeMaps::Request &req,
  slam_toolbox::MergeMaps::Response &resp)
/*****************************************************************************/
{
  ROS_INFO("Merging maps!");

  int id = 0;
  karto::LocalizedRangeScanVector transformed_scans;
  karto::LocalizedRangeScan* pScan;

  for(localized_range_scans_vec_it it_LRV = scans_vec_.begin();
    it_LRV != scans_vec_.end(); ++it_LRV)
  {
    id++;
    for (localized_range_scans_it iter = it_LRV->begin();
      iter != it_LRV->end(); ++iter)
    {
      pScan = *iter;
      tf2::Transform submapCorrection = submap_marker_transform_[id];

      // TRANSFORM BARYCENTERR POSE
      const karto::Pose2 baryCenterPose = pScan->GetBarycenterPose();
      auto baryCenterPoseCorr = applyCorrection(baryCenterPose, submapCorrection);
      pScan->SetBarycenterPose(baryCenterPoseCorr);

      // TRANSFORM BOUNDING BOX POSITIONS
      karto::BoundingBox2 bbox = pScan->GetBoundingBox();
      const karto::Vector2<kt_double> bboxMinCorr = applyCorrection(bbox.GetMinimum(),
        submapCorrection);
      bbox.SetMinimum(bboxMinCorr);
      const karto::Vector2<kt_double> bboxMaxCorr = applyCorrection(bbox.GetMaximum(),
        submapCorrection);
      bbox.SetMaximum(bboxMaxCorr);
      pScan->SetBoundingBox(bbox);

      // TRANSFORM UNFILTERED POINTS USED
      karto::PointVectorDouble UPRVec = pScan->GetPointReadings();
      for(karto::PointVectorDouble::iterator itUpr = UPRVec.begin();
        itUpr != UPRVec.end(); ++itUpr)
      {
        const karto::Vector2<kt_double> uprCorr = applyCorrection(*itUpr, submapCorrection);
        itUpr->SetX(uprCorr.GetX());
        itUpr->SetY(uprCorr.GetY());
      }
      pScan->SetPointReadings(UPRVec);

      // TRANSFORM CORRECTED POSE
      const karto::Pose2 correctedPose = pScan->GetCorrectedPose();
      karto::Pose2 kartoRobotPoseCorr = applyCorrection(correctedPose, submapCorrection);
      pScan->SetCorrectedPose(kartoRobotPoseCorr);
      kt_bool rIsDirty = false;
      pScan->SetIsDirty(rIsDirty);

      // TRANSFORM ODOM POSE
      karto::Pose2 odomPose = pScan->GetOdometricPose();
      karto::Pose2 kartoRobotPoseOdom = applyCorrection(odomPose, submapCorrection);
      pScan->SetOdometricPose(kartoRobotPoseOdom);
      transformed_scans.push_back(pScan);
    }
  }

  nav_msgs::GetMap::Response map;
  try
  {
    kartoToROSOccupancyGrid(transformed_scans, map);
  } catch (const karto::Exception & e)
  {
    ROS_WARN("Failed to build grid to merge maps together, Exception: %s",
      e.GetErrorMessage().c_str());
  }

  map.map.header.stamp = ros::Time::now();
  map.map.header.frame_id = "map";
  sstS_[0].publish(map.map);
  sstmS_[0].publish(map.map.info);
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
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP && 
      feedback->mouse_point_valid)
  {
    // we offset by 1
    const int id = std::stoi(feedback->marker_name,nullptr,10);

    // get yaw
    tfScalar yaw, pitch, roll;
    tf2::Quaternion quat(0.,0.,0.,1.0);
    tf2::convert(feedback->pose.orientation, quat); // relative
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
    tf2::Transform previous_submap_correction ;
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
    const int id = std::stoi(feedback->marker_name,nullptr,10);

    // get yaw
    tfScalar yaw, pitch, roll;
    tf2::Quaternion quat(0.,0.,0.,1.0);
    tf2::convert(feedback->pose.orientation, quat); // relative
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);

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
