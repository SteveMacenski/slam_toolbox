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

#include <slam_toolbox/merge_maps_optimization.hpp>
#include "serialization.cpp"

/*****************************************************************************/
MergeMapsOptimization::MergeMapsOptimization() : interactive_server_(NULL),
                                                 solver_loader_("slam_toolbox", "karto::ScanSolver")
/*****************************************************************************/
{
  ROS_INFO("MergeMapsOptimization: Starting up!");
  interactive_server_ = \
   new interactive_markers::InteractiveMarkerServer("merge_maps_optimization","",true);
  ros::NodeHandle nh_tmp("~");
  nh_ = nh_tmp;
  dataset_ = new karto::Dataset();
  SetConfigs();
}

/*****************************************************************************/
void MergeMapsOptimization::SetConfigs()
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
  sstmS_.push_back(nh_.advertise<nav_msgs::MapMetaData>( \
                                                    "/map_metadata", 1, true));
  ros::NodeHandle nh;
  ssMap_ = nh.advertiseService("merge_maps",
                                &MergeMapsOptimization::MergeMapCallback, this);
  ssSubmap_ = nh.advertiseService("add_submap",
                                   &MergeMapsOptimization::AddSubmapCallback, this);
  num_submaps_ = 0;
  tfB_ = new tf::TransformBroadcaster();
  std::string solver_plugin;
  if(!nh_.getParam("solver_plugin", solver_plugin))
  {
    ROS_WARN("unable to find requested solver plugin, defaulting to SPA");
    solver_plugin = "solver_plugins::SpaSolver";
  }
  try
  {
    master_solver_ = solver_loader_.createInstance(solver_plugin);
    ROS_INFO("Using plugin %s", solver_plugin.c_str());
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL("Failed to create %s, is it registered and built? Exception: %s.",
              solver_plugin.c_str(), ex.what());
    exit(1);
  }
  ROS_INFO("MergeMapsOptimization: Starting up!");
}

/*****************************************************************************/
MergeMapsOptimization::~MergeMapsOptimization()
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
  for(std::vector<karto::Mapper*>::iterator mapper_it = mapper_vec_.begin(); mapper_it != mapper_vec_.end(); ++mapper_it )
  {
    delete *mapper_it;
  }
}

/*****************************************************************************/
bool MergeMapsOptimization::AddSubmapCallback(slam_toolbox::AddSubmap::Request &req,
                                     slam_toolbox::AddSubmap::Response &resp)
/*****************************************************************************/
{
  // find if file exists
  const std::string filename = req.filename ;
  karto::Mapper* mapper = new karto::Mapper;
  karto::Dataset* dataset = new karto::Dataset;
  serialization::Read(filename, mapper, dataset);
  karto::LaserRangeFinder* laser = dynamic_cast<karto::LaserRangeFinder*>(dataset->GetObjects()[0]);
  if (lasers_.find(laser->GetName().GetName())==lasers_.end())
  {
    lasers_[laser->GetName().GetName()] = laser;
    karto::Sensor* pSensor = dynamic_cast<karto::Sensor *>(laser);
    if (pSensor != NULL)
    {
      karto::SensorManager::GetInstance()->RegisterSensor(pSensor);
    }
  }
  RecalculateSolver(mapper);
  mapper_vec_.push_back(mapper);
  dataset_vec_.push_back(dataset);
  karto::LocalizedRangeScanVector scans = mapper->GetAllProcessedScans();
  scans_vec_.push_back(scans);
  num_submaps_++;

  // create and publish map with marker that will move the map around
  sstS_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>( \
                            "/map_"+std::to_string(num_submaps_), 1, true));
  sstmS_.push_back(nh_.advertise<nav_msgs::MapMetaData>( \
                 "/map_metadata_" + std::to_string(num_submaps_), 1, true));
  ros::Duration(1.).sleep();
  nav_msgs::GetMap::Response map;
  KartoToROSOccupancyGrid(scans,map);

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
           boost::bind(&MergeMapsOptimization::ProcessInteractiveFeedback, this, _1));
  interactive_server_->applyChanges();
  delete mapper;
  mapper = NULL;
  return true;
}

/*****************************************************************************/
bool MergeMapsOptimization::MergeMapCallback(slam_toolbox::MergeMaps::Request &req,
                                    slam_toolbox::MergeMaps::Response &resp)
/*****************************************************************************/
{
  int id = 0;
  karto::Pose2 corrected_pose;
  karto::LocalizedRangeScan* pScan;
  karto::LocalizedRangeScanVector transformed_scans;
  std::vector<karto::LocalizedRangeScanVector>& vector_of_scans = scans_vec_;
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
      Pose2_ptr karto_baryCenterPose_corr = std::move(ApplyCorrection(baryCenter_pose, submap_correction));
      pScan_copy->SetBarycenterPose(*karto_baryCenterPose_corr);

      //TRANSFORM BOUNDING BOX POSITIONS
      karto::BoundingBox2 bbox = pScan_copy->GetBoundingBox();
      const Vector2_double_ptr bbox_min_corr = std::move(ApplyCorrection(bbox.GetMinimum(), submap_correction));
      bbox.SetMinimum(*bbox_min_corr);
      const Vector2_double_ptr bbox_max_corr =std::move(ApplyCorrection(bbox.GetMaximum(), submap_correction));
      bbox.SetMaximum(*bbox_max_corr);
      pScan_copy->SetBoundingBox(bbox);

      // TRANSFORM UNFILTERED POINTS USED
      karto::PointVectorDouble UPR_vec = pScan_copy->GetPointReadings();
      for(karto::PointVectorDouble::iterator it_upr = UPR_vec.begin(); it_upr!=UPR_vec.end(); ++it_upr)
      {
        const Vector2_double_ptr upr_corr = std::move(ApplyCorrection(*it_upr, submap_correction));
        (*it_upr).SetX(upr_corr->GetX());
        (*it_upr).SetY(upr_corr->GetY());
      }
      pScan_copy->SetPointReadings(UPR_vec);

      //TRANSFORM CORRECTED POSE
      corrected_pose = pScan_copy->GetCorrectedPose();
      Pose2_ptr karto_robotPose_corr = std::move(ApplyCorrection(corrected_pose, submap_correction));
      pScan_copy->SetCorrectedPose(*karto_robotPose_corr);
      kt_bool rIsDirty = false;
      pScan_copy->SetIsDirty(rIsDirty);

      //TRANSFORM ODOM POSE
      karto::Pose2 odom_pose = pScan_copy->GetOdometricPose();
      Pose2_ptr karto_robotPose_odom = std::move(ApplyCorrection(odom_pose, submap_correction));
      pScan_copy->SetOdometricPose(*karto_robotPose_odom);
      transformed_scans.push_back(pScan_copy);
    }
  }
  nav_msgs::GetMap::Response map;
  KartoToROSOccupancyGrid(transformed_scans, map);
  map.map.header.stamp = ros::Time::now();
  map.map.header.frame_id = "map";
  sstS_[0].publish(map.map);
  sstmS_[0].publish(map.map.info);
}
/*****************************************************************************/
void MergeMapsOptimization::KartoToROSOccupancyGrid( \
                                  const karto::LocalizedRangeScanVector& scans, nav_msgs::GetMap::Response& map)
/*****************************************************************************/
{
  karto::OccupancyGrid* occ_grid = NULL;
  occ_grid = karto::OccupancyGrid::CreateFromScans(scans, resolution_);
  if (!occ_grid)
  {
    ROS_INFO("MergeMapsOptimization: Could not make Karto occupancy grid.");
    return;
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
  delete occ_grid;
  return;
}

/*****************************************************************************/
void MergeMapsOptimization::ProcessInteractiveFeedback(const \
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
void MergeMapsOptimization::RecalculateSolver(karto::Mapper* mapper)
/*****************************************************************************/
{
  std::map<karto::Name, std::vector<karto::Vertex<karto::LocalizedRangeScan>*>> mapper_vertices = mapper->GetGraph()->GetVertices();
  std::map<karto::Name, std::vector<karto::Vertex<karto::LocalizedRangeScan>*>>::iterator vertices_it;
  for(vertices_it = mapper_vertices.begin(); vertices_it!=mapper_vertices.end(); ++vertices_it)
  {
    for(std::vector<karto::Vertex<karto::LocalizedRangeScan>*>::iterator vertex_it=  vertices_it->second.begin();
        vertex_it!= vertices_it->second.end();++vertex_it )
    {
     master_solver_->AddNode(*vertex_it);
    }
  }
  std::vector<karto::Edge<karto::LocalizedRangeScan>*> mapper_edges = mapper->GetGraph()->GetEdges();
  std::vector<karto::Edge<karto::LocalizedRangeScan>*>::iterator edges_it;
  for( edges_it = mapper_edges.begin(); edges_it != mapper_edges.end(); ++edges_it)
  {
    master_solver_->AddConstraint(*edges_it);
  }
//  mapper->SetScanSolver(master_solver_.get());
}
/*****************************************************************************/
int main(int argc, char** argv)
/*****************************************************************************/
{
  ros::init(argc, argv, "merge_maps_optimization");
  MergeMapsOptimization mmt;
  ros::spin();
  return 0;
}
