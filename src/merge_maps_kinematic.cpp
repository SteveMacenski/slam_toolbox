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
MergeMapsKinematic::MergeMapsKinematic() : nh_("~")
/*****************************************************************************/
{
  ROS_INFO("MergeMapsKinematic: Starting up!");
  Setup();
}

/*****************************************************************************/
void MergeMapsKinematic::Setup()
/*****************************************************************************/
{
  if(!nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!nh_.getParam("max_laser_range", max_laser_range_))
    max_laser_range_ = 25.0;
  if(!nh_.getParam("resolution", resolution_))
  {
    resolution_ = 0.05;
  }
  sstS_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true));
  sstmS_.push_back(nh_.advertise<nav_msgs::MapMetaData>(
    "/map_metadata", 1, true));
  ros::NodeHandle nh("map_merging");
  ssMap_ = nh.advertiseService("merge_submaps",
    &MergeMapsKinematic::MergeMapCallback, this);
  ssSubmap_ = nh.advertiseService("add_submap",
    &MergeMapsKinematic::AddSubmapCallback, this);
  num_submaps_ = 0;
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
bool MergeMapsKinematic::AddSubmapCallback(
  slam_toolbox::AddSubmap::Request &req,
  slam_toolbox::AddSubmap::Response &resp)
/*****************************************************************************/
{
  std::unique_ptr<karto::Mapper> mapper = std::make_unique<karto::Mapper>();
  std::unique_ptr<karto::Dataset> dataset = std::make_unique<karto::Dataset>();

  serialization::Read(req.filename, *mapper, *dataset);

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
  try
  {
    KartoToROSOccupancyGrid(scans, map);
  } catch (const karto::Exception & e)
  {
    ROS_WARN("Failed to build grid to add submap, Exception: %s",
      e.GetErrorMessage().c_str());
    return false;
  }

  tf2::Transform transform;
  transform.setIdentity();
  transform.setOrigin(tf2::Vector3(map.map.info.origin.position.x +
    map.map.info.width * map.map.info.resolution / 2.0,
    map.map.info.origin.position.y +
    map.map.info.height * map.map.info.resolution / 2.0,
    0.));
  map.map.info.origin.position.x = - (map.map.info.width * map.map.info.resolution / 2.0);
  map.map.info.origin.position.y = - (map.map.info.height * map.map.info.resolution / 2.0);
  map.map.header.stamp = ros::Time::now();
  map.map.header.frame_id = "map_"+std::to_string(num_submaps_);
  sstS_[num_submaps_].publish(map.map);
  sstmS_[num_submaps_].publish(map.map.info);
  // tfB_->sendTransform(tf::StampedTransform (transform, ros::Time::now(),
  //   "/map", "/map_"+std::to_string(num_submaps_)));
  submap_marker_transform_[num_submaps_]=tf2::Transform(tf2::Quaternion(0.,0.,0.,1.0),
    tf2::Vector3(0,0,0)); //no initial correction -- identity mat

  // create an interactive marker for the base of this frame and attach it
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "merge_maps_tool";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.z = 0.0;
  m.pose.position.x =  transform.getOrigin().getX();
  m.pose.position.y =  transform.getOrigin().getY();
  submap_locations_[num_submaps_] = Eigen::Vector3d(transform.getOrigin().getX(),
    transform.getOrigin().getY(),0.);
  m.pose.orientation.w = 1.;
  m.scale.x = 2; m.scale.y = 2; m.scale.z = 2;
  m.color.r = 1.0; m.color.g = 0; m.color.b = 0.0; m.color.a = 1.;
  m.action = visualization_msgs::Marker::ADD;
  m.lifetime = ros::Duration(0.);
  m.id = num_submaps_;

  // marker and metadata
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = std::to_string(num_submaps_);
  int_marker.pose.orientation.w = 1.;
  int_marker.pose.position.x = m.pose.position.x;
  int_marker.pose.position.y = m.pose.position.y;
  int_marker.scale = 2.4;

  // translate control
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode =
    visualization_msgs::InteractiveMarkerControl::FIXED;
  control.always_visible = true;
  control.orientation.w = 0;
  control.orientation.x = 0.7071;
  control.orientation.y = 0;
  control.orientation.z = 0.7071;
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.markers.push_back( m );
  int_marker.controls.push_back( control );

  // rotate control
  visualization_msgs::InteractiveMarkerControl control_rot;
  control_rot.orientation_mode =
    visualization_msgs::InteractiveMarkerControl::FIXED;
  control_rot.always_visible = true;
  control_rot.orientation.w = 0;
  control_rot.orientation.x = 0.7071;
  control_rot.orientation.y = 0;
  control_rot.orientation.z = 0.7071;
  control_rot.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back( control_rot );

  interactive_server_->insert(int_marker,
    boost::bind(&MergeMapsKinematic::ProcessInteractiveFeedback, this, _1));
  interactive_server_->applyChanges();
  ROS_INFO("Map %s was loaded into topic %s!", req.filename.c_str(),
    ("/map_"+std::to_string(num_submaps_)).c_str());
  return true;
}

/*****************************************************************************/
karto::Pose2 MergeMapsKinematic::ApplyCorrection(const
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
karto::Vector2<kt_double> MergeMapsKinematic::ApplyCorrection(const
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
bool MergeMapsKinematic::MergeMapCallback(
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
      auto baryCenterPoseCorr = ApplyCorrection(baryCenterPose, submapCorrection);
      pScan->SetBarycenterPose(baryCenterPoseCorr);

      // TRANSFORM BOUNDING BOX POSITIONS
      karto::BoundingBox2 bbox = pScan->GetBoundingBox();
      const karto::Vector2<kt_double> bboxMinCorr = ApplyCorrection(bbox.GetMinimum(),
        submapCorrection);
      bbox.SetMinimum(bboxMinCorr);
      const karto::Vector2<kt_double> bboxMaxCorr = ApplyCorrection(bbox.GetMaximum(),
        submapCorrection);
      bbox.SetMaximum(bboxMaxCorr);
      pScan->SetBoundingBox(bbox);

      // TRANSFORM UNFILTERED POINTS USED
      karto::PointVectorDouble UPRVec = pScan->GetPointReadings();
      for(karto::PointVectorDouble::iterator itUpr = UPRVec.begin();
        itUpr != UPRVec.end(); ++itUpr)
      {
        const karto::Vector2<kt_double> uprCorr = ApplyCorrection(*itUpr, submapCorrection);
        itUpr->SetX(uprCorr.GetX());
        itUpr->SetY(uprCorr.GetY());
      }
      pScan->SetPointReadings(UPRVec);

      // TRANSFORM CORRECTED POSE
      const karto::Pose2 correctedPose = pScan->GetCorrectedPose();
      karto::Pose2 kartoRobotPoseCorr = ApplyCorrection(correctedPose, submapCorrection);
      pScan->SetCorrectedPose(kartoRobotPoseCorr);
      kt_bool rIsDirty = false;
      pScan->SetIsDirty(rIsDirty);

      // TRANSFORM ODOM POSE
      karto::Pose2 odomPose = pScan->GetOdometricPose();
      karto::Pose2 kartoRobotPoseOdom = ApplyCorrection(odomPose, submapCorrection);
      pScan->SetOdometricPose(kartoRobotPoseOdom);
      transformed_scans.push_back(pScan);
    }
  }

  nav_msgs::GetMap::Response map;
  try
  {
    KartoToROSOccupancyGrid(transformed_scans, map);
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
void MergeMapsKinematic::KartoToROSOccupancyGrid(
  const karto::LocalizedRangeScanVector& scans,
  nav_msgs::GetMap::Response& map)
/*****************************************************************************/
{
  karto::OccupancyGrid* occ_grid = NULL;
  occ_grid = karto::OccupancyGrid::CreateFromScans(scans, resolution_);
  if (!occ_grid)
  {
    ROS_INFO("MergeMapsKinematic: Could not make Karto occupancy grid.");
    delete occ_grid;
    return;
  }
  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
    occ_grid->GetCoordinateConverter()->GetOffset();

  if(map.map.info.width != (unsigned int) width ||
     map.map.info.height != (unsigned int) height ||
     map.map.info.origin.position.x != offset.GetX() ||
     map.map.info.origin.position.y != offset.GetY())
  {
    map.map.info.origin.position.x = offset.GetX();
    map.map.info.origin.position.y = offset.GetY();
    map.map.info.origin.position.z = 0.0;
    map.map.info.origin.orientation.w = 1.0;
    map.map.info.width = width;
    map.map.info.height = height;
    map.map.info.resolution = resolution_;
    map.map.data.resize(map.map.info.width * map.map.info.height);
  }

  for (kt_int32s y=0; y<height; y++)
  {
    for (kt_int32s x=0; x<width; x++) 
    {
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
      switch (value)
      {
        case karto::GridStates_Unknown:
          map.map.data[MAP_IDX(map.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map.map.data[MAP_IDX(map.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map.map.data[MAP_IDX(map.map.info.width, x, y)] = 0;
          break;
        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }
  delete occ_grid;
  return;
}

/*****************************************************************************/
void MergeMapsKinematic::ProcessInteractiveFeedback(const
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
    // tfB_->sendTransform(tf::StampedTransform (new_submap_location,
    //   ros::Time::now(), "/map", "/map_"+std::to_string(id)));

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
    // tfB_->sendTransform(tf::StampedTransform (new_submap_location,
    //   ros::Time::now(), "/map", "/map_"+std::to_string(id))); //TODO util to poulate. done 5x
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
