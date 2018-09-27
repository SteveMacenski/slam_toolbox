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

#include <slam_toolbox/merge_maps_tool.hpp>
#include "serialization.cpp"

/*****************************************************************************/
MergeMapTool::MergeMapTool() : interactive_server_(NULL)
/*****************************************************************************/
{
  ROS_INFO("MergeMapTool: Starting up!");
  interactive_server_ = \
   new interactive_markers::InteractiveMarkerServer("merge_maps_tool","",true);

  ros::NodeHandle nh_tmp("~");
  nh_ = nh_tmp;
  dataset_ = new karto::Dataset();
  SetConfigs();
}

/*****************************************************************************/
void MergeMapTool::SetConfigs()
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
  karto::LaserRangeFinder* laser =
          karto::LaserRangeFinder::CreateLaserRangeFinder( \
                                              karto::LaserRangeFinder_Custom, \
                                         karto::Name("Custom Described Lidar"));
  dataset_->Add(laser);
  sstS_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true));
  sstmS_.push_back(nh_.advertise<nav_msgs::MapMetaData>( \
                                                    "/map_metadata", 1, true));
  ssMap_ = nh_.advertiseService("merge_maps", 
                                        &MergeMapTool::MergeMapCallback, this);
  ssSubmap_ = nh_.advertiseService("add_submap", 
                                       &MergeMapTool::AddSubmapCallback, this);
  num_submaps_ = 0;
  tfB_ = new tf::TransformBroadcaster();
}

/*****************************************************************************/
MergeMapTool::~MergeMapTool()
/*****************************************************************************/
{
  if (interactive_server_)
  {
    delete interactive_server_;
  }
  if (dataset_)
  {
    delete dataset_;
  }
}

/*****************************************************************************/
bool MergeMapTool::AddSubmapCallback(slam_toolbox::AddSubmap::Request &req,
                                     slam_toolbox::AddSubmap::Response &resp)
/*****************************************************************************/
{
  // find if file exists
  const std::string filename = req.filename + std::string(".st");
  if (!FileExists(filename))
  {
    ROS_ERROR("MergeMapTool: Failed to open requested submap %s.", filename.c_str());
    return true;
  }
  karto::Mapper* mapper = new karto::Mapper;
  serialization::Read(filename, mapper);
  karto::LocalizedRangeScanVector scans = mapper->GetAllProcessedScans();
  scans_vec_.push_back(scans);
  num_submaps_++;

  // create and publish map with marker that will move the map around
  sstS_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>( \
                            "/map_"+std::to_string(num_submaps_), 1, true));
  sstmS_.push_back(nh_.advertise<nav_msgs::MapMetaData>( \
                 "/map_metadata_" + std::to_string(num_submaps_), 1, true));
  ros::Duration(1.).sleep();
  nav_msgs::GetMap::Response map = KartoToROSOccupancyGrid(scans);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(map.map.info.origin.position.x + map.map.info.width * map.map.info.resolution / 2.0,\
                                  map.map.info.origin.position.y + map.map.info.height * map.map.info.resolution / 2.0, 0.));
  map.map.info.origin.position.x = - (map.map.info.width * map.map.info.resolution / 2.0);
  map.map.info.origin.position.y = - (map.map.info.height * map.map.info.resolution / 2.0);
  transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
  map.map.header.stamp = ros::Time::now();
  map.map.header.frame_id = "map_"+std::to_string(num_submaps_);
  sstS_[num_submaps_].publish(map.map);
  sstmS_[num_submaps_].publish(map.map.info);
  tfB_->sendTransform(tf::StampedTransform (transform, ros::Time::now(), "/map", "/map_"+std::to_string(num_submaps_)));
  submap_marker_transform_[num_submaps_]=tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                     tf::Vector3(0,0,0));//no initial correction -- identity mat

  // create an interactive marker for the base of this frame and attach it
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "merge_maps_tool";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.z = 0.0;
  m.pose.position.x =  transform.getOrigin().getX();
  m.pose.position.y =  transform.getOrigin().getY();
  submap_locations_[num_submaps_] = Eigen::Vector3d(transform.getOrigin().getX(),transform.getOrigin().getY(),0.);
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
  control.orientation_mode = \
                          visualization_msgs::InteractiveMarkerControl::FIXED;
  control.always_visible = true;
  control.orientation.w = 0;
  control.orientation.x = 0.7071;
  control.orientation.y = 0;
  control.orientation.z = 0.7071;
  control.interaction_mode = \
                   visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.markers.push_back( m );
  int_marker.controls.push_back( control );

  // rotate control
  visualization_msgs::InteractiveMarkerControl control_rot;
  control_rot.orientation_mode = \
                          visualization_msgs::InteractiveMarkerControl::FIXED;
  control_rot.always_visible = true;
  control_rot.orientation.w = 0;
  control_rot.orientation.x = 0.7071;
  control_rot.orientation.y = 0;
  control_rot.orientation.z = 0.7071;
  control_rot.interaction_mode = \
                   visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back( control_rot );

  interactive_server_->insert(int_marker, \
           boost::bind(&MergeMapTool::ProcessInteractiveFeedback, this, _1));
  interactive_server_->applyChanges();
  delete mapper;
  mapper = NULL;
  return true;
}

/*****************************************************************************/
bool MergeMapTool::MergeMapCallback(slam_toolbox::MergeMaps::Request &req,
                                    slam_toolbox::MergeMaps::Response &resp)
/*****************************************************************************/
{
    int id = 0;
    karto::Pose2 corrected_pose;
    karto::LocalizedRangeScan* pScan;
    karto::LocalizedRangeScanVector transformed_scans;
    std::vector<karto::LocalizedRangeScanVector> vector_of_scans = scans_vec_;
    karto::LocalizedRangeScan* pScan_copy;

    for(LocalizedRangeScansVec_it it_LRV = vector_of_scans.begin(); it_LRV!= vector_of_scans.end(); ++it_LRV)
    {
      id++;
      for ( LocalizedRangeScans_it iter = (*it_LRV).begin(); iter != (*it_LRV).end();++iter)
      {
        pScan= *iter;
        pScan_copy = pScan;
        tf::Transform submap_correction = submap_marker_transform_[id];

        //TRANSFORM BARYCENTERR POSE
        const karto::Pose2 baryCenter_pose = pScan_copy->GetBarycenterPose();
        karto::Pose2 karto_baryCenterPose_corr = ApplyCorrection(baryCenter_pose, submap_correction);
        pScan_copy->SetBarycenterPose(karto_baryCenterPose_corr);

        //TRANSFORM BOUNDING BOX POSITIONS
        karto::BoundingBox2 bbox = pScan_copy->GetBoundingBox();
        const karto::Vector2<kt_double> bbox_min_corr = ApplyCorrection(bbox.GetMinimum(), submap_correction);
        bbox.SetMinimum(bbox_min_corr);
        const karto::Vector2<kt_double> bbox_max_corr = ApplyCorrection(bbox.GetMaximum(), submap_correction);
        bbox.SetMaximum(bbox_max_corr);
        pScan_copy->SetBoundingBox(bbox);

        // TRANSFORM UNFILTERED POINTS USED
        karto::PointVectorDouble UPR_vec = pScan_copy->GetPointReadings();
        for(karto::PointVectorDouble::iterator it_upr = UPR_vec.begin(); it_upr!=UPR_vec.end(); ++it_upr)
        {
          const karto::Vector2<kt_double>& upr_corr = ApplyCorrection(*it_upr, submap_correction);
          (*it_upr).SetX(upr_corr.GetX());
          (*it_upr).SetY(upr_corr.GetY());
        }
        pScan_copy->SetPointReadings(UPR_vec);

        //TRANSFORM CORRECTED POSE
        corrected_pose = pScan_copy->GetCorrectedPose();
        karto::Pose2 karto_robotPose_corr = ApplyCorrection(corrected_pose, submap_correction);
        pScan_copy->SetCorrectedPose(karto_robotPose_corr);
        kt_bool rIsDirty = false;
        pScan_copy->SetIsDirty(rIsDirty);

        //TRANSFORM ODOM POSE
        karto::Pose2 odom_pose = pScan_copy->GetOdometricPose();
        karto::Pose2 karto_robotPose_odom = ApplyCorrection(odom_pose, submap_correction);
        pScan_copy->SetOdometricPose(karto_robotPose_odom);

        transformed_scans.push_back(pScan_copy);
      }
    }
    nav_msgs::GetMap::Response map = KartoToROSOccupancyGrid(transformed_scans);
    map.map.header.stamp = ros::Time::now();
    map.map.header.frame_id = "map";
    sstS_[0].publish(map.map);
    sstmS_[0].publish(map.map.info);
}

/*****************************************************************************/
nav_msgs::GetMap::Response MergeMapTool::KartoToROSOccupancyGrid( \
                                  const karto::LocalizedRangeScanVector& scans)
/*****************************************************************************/
{
  karto::OccupancyGrid* occ_grid = NULL;
  nav_msgs::GetMap::Response map;
  occ_grid = karto::OccupancyGrid::CreateFromScans(scans, resolution_);

  if (!occ_grid)
  {
    ROS_INFO("MergeMapTool: Could not make Karto occupancy grid.");
    return map;
  }
  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = \
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
  return map;
}

/*****************************************************************************/
void MergeMapTool::ProcessInteractiveFeedback(const \
               visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
/*****************************************************************************/
{
  if (feedback->event_type == \
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP && 
      feedback->mouse_point_valid)
  {
    // we offset by 1
    const int id = std::stoi(feedback->marker_name,nullptr,10);

    // get yaw
    tfScalar yaw, pitch, roll;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(feedback->pose.orientation, quat); // relative
    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
    tf::Transform previous_submap_correction ;
    previous_submap_correction.setOrigin(tf::Vector3(submap_locations_[id](0),submap_locations_[id](1), 0.));
    previous_submap_correction.setRotation(tf::createQuaternionFromRPY(0, 0, 0));

    // update internal knowledge of submap locations
    submap_locations_[id] = Eigen::Vector3d(feedback->pose.position.x, \
                                            feedback->pose.position.y, \
                                            submap_locations_[id](2) + yaw);

    // add the map_N frame there
    tf::Transform new_submap_location;
    new_submap_location.setOrigin(tf::Vector3(submap_locations_[id](0), \
                                    submap_locations_[id](1), 0.));
    new_submap_location.setRotation(quat);
    tfB_->sendTransform(tf::StampedTransform (new_submap_location,
                       ros::Time::now(), "/map", "/map_"+std::to_string(id)));

    submap_marker_transform_[id] = submap_marker_transform_[id] * previous_submap_correction.inverse() * new_submap_location;
  }

  if (feedback->event_type == \
                   visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    const int id = std::stoi(feedback->marker_name,nullptr,10);

    // get yaw
    tfScalar yaw, pitch, roll;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(feedback->pose.orientation, quat); // relative
    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);

    // add the map_N frame there
    tf::Transform new_submap_location;
    new_submap_location.setOrigin(tf::Vector3(feedback->pose.position.x, \
                                    feedback->pose.position.y, 0.));
    new_submap_location.setRotation(quat);
    tfB_->sendTransform(tf::StampedTransform (new_submap_location,
                       ros::Time::now(), "/map", "/map_"+std::to_string(id)));
  }
}

/*****************************************************************************/
int main(int argc, char** argv)
/*****************************************************************************/
{
  ros::init(argc, argv, "merge_map_tool");
  MergeMapTool mmt;
  ros::spin();
  return 0;
}
