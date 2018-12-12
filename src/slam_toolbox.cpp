/*
 * slam_toolbox
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
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

/* Author: Brian Gerkey */
/* Heavily Modified: Steven Macenski */

#include <slam_toolbox/slam_toolbox.hpp>
#include "serialization.cpp"
/*****************************************************************************/
SlamToolbox::SlamToolbox() : 
                         transform_thread_(NULL),
                         run_thread_(NULL),
                         visualization_thread_(NULL),
                         mapper_(NULL),
                         dataset_(NULL),
                         scan_filter_sub_(NULL),
                         tfB_(NULL),
                         interactive_server_(NULL),
                         solver_loader_("slam_toolbox", "karto::ScanSolver"),
                         pause_graph_(false),
                         pause_processing_(false),
                         pause_new_measurements_(false),
                         interactive_mode_(false),
                         laser_count_(0),
                         tf_(ros::Duration(14400.)) // 4 hours
/*****************************************************************************/
{
  interactive_server_ = \
      new interactive_markers::InteractiveMarkerServer("slam_toolbox","",true);

  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  ros::NodeHandle private_nh("~");
  nh_ = private_nh;
  SetParams(private_nh);

  SetSolver(private_nh);
  SetROSInterfaces(private_nh);

  nh_.setParam("paused_processing", pause_processing_);
  nh_.setParam("paused_new_measurements", pause_new_measurements_);
  nh_.setParam("interactive_mode", interactive_mode_);

  double transform_publish_period;
  private_nh.param("transform_publish_period", 
                                               transform_publish_period, 0.05);
  transform_thread_ = new boost::thread(boost::bind(&SlamToolbox::PublishLoop, 
                                              this, transform_publish_period));
  run_thread_ = new boost::thread(boost::bind(&SlamToolbox::Run, this));
  visualization_thread_ = new boost::thread(\
                       boost::bind(&SlamToolbox::PublishVisualizations, this));
}

/*****************************************************************************/
void SlamToolbox::SetSolver(ros::NodeHandle& private_nh_)
/*****************************************************************************/
{
  // Set solver to be used in loop closure
  std::string solver_plugin;
  if(!private_nh_.getParam("solver_plugin", solver_plugin))
  {
    ROS_WARN("unable to find requested solver plugin, defaulting to SPA");
    solver_plugin = "solver_plugins::SpaSolver";
  }
  try 
  {
    solver_ = solver_loader_.createInstance(solver_plugin);
    ROS_INFO("Using plugin %s", solver_plugin.c_str());
  } 
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL("Failed to create %s, is it registered and built? Exception: %s.", 
                                            solver_plugin.c_str(), ex.what());
    exit(1);
  }
  mapper_->SetScanSolver(solver_.get());
}

/*****************************************************************************/
void SlamToolbox::SetParams(ros::NodeHandle& private_nh_)
/*****************************************************************************/
{
  map_to_odom_.setIdentity();

  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_footprint";
  if(!private_nh_.getParam("laser_frame", laser_frame_))
    laser_frame_ = "base_laser_link";
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("publish_occupancy_map", publish_occupancy_map_))
    publish_occupancy_map_ = false;
  if(!private_nh_.getParam("max_laser_range", max_laser_range_))
    max_laser_range_ = 25.0;
  if(!private_nh_.getParam("minimum_time_interval", minimum_time_interval_))
    minimum_time_interval_ = 0.5;
  double map_update_interval;
  if(!private_nh_.getParam("map_update_interval", map_update_interval))
    map_update_interval = 10.0;
  map_update_interval_.fromSec(map_update_interval);
  if(!private_nh_.getParam("resolution", resolution_))
  {
    resolution_ = 0.05;
  }
  bool debug = false;
  if (private_nh_.getParam("debug_logging", debug) && debug)
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();   
    }
  }
  // Setting General Parameters
  bool use_scan_matching;
  if(private_nh_.getParam("use_scan_matching", use_scan_matching))
    mapper_->setParamUseScanMatching(use_scan_matching);
  
  bool use_scan_barycenter;
  if(private_nh_.getParam("use_scan_barycenter", use_scan_barycenter))
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  minimum_travel_distance_ = 0.5;
  if(private_nh_.getParam("minimum_travel_distance", minimum_travel_distance_))
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance_);

  double minimum_travel_heading;
  if(private_nh_.getParam("minimum_travel_heading", minimum_travel_heading))
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size;
  if(private_nh_.getParam("scan_buffer_size", scan_buffer_size))
    mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance;
  if(private_nh_.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine;
  if(private_nh_.getParam("link_match_minimum_response_fine", link_match_minimum_response_fine))
    mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance;
  if(private_nh_.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance;
  if(private_nh_.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing;
  if(private_nh_.getParam("do_loop_closing", do_loop_closing))
    mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size;
  if(private_nh_.getParam("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse;
  if(private_nh_.getParam("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
    mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse;
  if(private_nh_.getParam("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
    mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine;
  if(private_nh_.getParam("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
    mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters
  double correlation_search_space_dimension;
  if(private_nh_.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution;
  if(private_nh_.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation;
  if(private_nh_.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension;
  if(private_nh_.getParam("loop_search_space_dimension", loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if(private_nh_.getParam("loop_search_space_resolution", loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation;
  if(private_nh_.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters
  double distance_variance_penalty;
  if(private_nh_.getParam("distance_variance_penalty", distance_variance_penalty))
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty;
  if(private_nh_.getParam("angle_variance_penalty", angle_variance_penalty))
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset;
  if(private_nh_.getParam("fine_search_angle_offset", fine_search_angle_offset))
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset;
  if(private_nh_.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution;
  if(private_nh_.getParam("coarse_angle_resolution", coarse_angle_resolution))
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty;
  if(private_nh_.getParam("minimum_angle_penalty", minimum_angle_penalty))
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty;
  if(private_nh_.getParam("minimum_distance_penalty", minimum_distance_penalty))
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion;
  if(private_nh_.getParam("use_response_expansion", use_response_expansion))
    mapper_->setParamUseResponseExpansion(use_response_expansion);
}

/*****************************************************************************/
void SlamToolbox::SetROSInterfaces(ros::NodeHandle& node)
/*****************************************************************************/
{
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  sstm_ = node.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, true);
  ssMap_ = node.advertiseService("dynamic_map", &SlamToolbox::MapCallback, this);
  ssClear_ = node.advertiseService("clear_queue", &SlamToolbox::ClearQueueCallback, this);
  ssPause_processing_ = node.advertiseService("pause_processing", &SlamToolbox::PauseProcessingCallback, this);
  ssPause_measurements_ = node.advertiseService("pause_new_measurements", &SlamToolbox::PauseNewMeasurementsCallback, this);
  ssLoopClosure_ = node.advertiseService("manual_loop_closure", &SlamToolbox::ManualLoopClosureCallback, this);
  ssInteractive_ = node.advertiseService("toggle_interactive_mode", &SlamToolbox::InteractiveCallback,this);
  ssClear_manual_ = node.advertiseService("clear_changes", &SlamToolbox::ClearChangesCallback, this);
  ssSave_map_ = node.advertiseService("save_map", &SlamToolbox::SaveMapCallback, this);
  ssSerialize_ = node.advertiseService("serialize_map", &SlamToolbox::SerializePoseGraphCallback, this);
  ssLoadMap_ = node.advertiseService("load_map", &SlamToolbox::LoadMapperCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node, "/scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamToolbox::LaserCallback, this, _1));
  marker_publisher_ = node.advertise<visualization_msgs::MarkerArray>("karto_graph_visualization",1);
  scan_publisher_ = node.advertise<sensor_msgs::LaserScan>("karto_scan_visualization",10);
}

/*****************************************************************************/
SlamToolbox::~SlamToolbox()
/*****************************************************************************/
{
  if(transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if(run_thread_)
  {
    run_thread_->join();
    delete run_thread_;
  }
  if(visualization_thread_)
  {
    visualization_thread_->join();
    delete visualization_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
  if (mapper_)
    delete mapper_;
  if (dataset_)
    delete dataset_;
  std::map<std::string, karto::LaserRangeFinder*>::iterator it = \
                                                               lasers_.begin();
  for (it; it!=lasers_.end(); ++it)
  {
    delete it->second;
  }
  if (interactive_server_)
  {
    delete interactive_server_;
  }
}

/*****************************************************************************/
void SlamToolbox::PublishLoop(double transform_publish_period)
/*****************************************************************************/
{
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok())
  {
    {
      boost::mutex::scoped_lock lock(map_to_odom_mutex_);
      ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
      tfB_->sendTransform(tf::StampedTransform (map_to_odom_, 
                                   ros::Time::now(), map_frame_, odom_frame_));
    }
    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::PublishVisualizations()
/*****************************************************************************/
{
  map_.map.info.resolution = resolution_;
  map_.map.info.origin.position.x = 0.0;
  map_.map.info.origin.position.y = 0.0;
  map_.map.info.origin.position.z = 0.0;
  map_.map.info.origin.orientation.x = 0.0;
  map_.map.info.origin.orientation.y = 0.0;
  map_.map.info.origin.orientation.z = 0.0;
  map_.map.info.origin.orientation.w = 1.0;

  ros::Rate r(1.0 / map_update_interval_.toSec());

  while(ros::ok())
  {
    UpdateMap();
    if(!IsPaused(VISUALIZING_GRAPH))
    {
      PublishGraph();
    }
    r.sleep();
  }

}

/*****************************************************************************/
karto::LaserRangeFinder* SlamToolbox::GetLaser(const \
                                        sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  if(lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
	       e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_DEBUG("laser %s's pose wrt base: %.3f %.3f %.3f",
	           scan->header.frame_id.c_str(),
	           laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw);

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try
    {
      tf_.transformPoint(scan->header.frame_id, up, up);
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      ROS_DEBUG("laser is mounted upside-down");

    // Create a laser range finder device and copy in data from the first scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder( \
                                              karto::LaserRangeFinder_Custom, \
                                         karto::Name("Custom Described Lidar"));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
				      laser_pose.getOrigin().y(),
				      yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    laser->SetRangeThreshold(max_laser_range_);

    // Store this laser device for later
    if(lasers_.find(scan->header.frame_id) == lasers_.end())
    {
      lasers_[scan->header.frame_id] = laser;
      dataset_->Add(laser, true);
    }
  }
  return lasers_[scan->header.frame_id];
}

/*****************************************************************************/
bool SlamToolbox::GetOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
/*****************************************************************************/
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose = \
          karto::Pose2(odom_pose.getOrigin().x(), \
                       odom_pose.getOrigin().y(), yaw);
  return true;
}

/*****************************************************************************/
void SlamToolbox::PublishGraph()
/*****************************************************************************/
{
  std::vector<Eigen::Vector2d> graph;
  solver_->getGraph(graph);

  if (graph.size() == 0)
  {
    return;
  }

  ROS_INFO_THROTTLE(15.,"Graph size: %i",(int)graph.size());
  bool interactive_mode = false;
  {
    boost::mutex::scoped_lock lock(interactive_mutex_);
    interactive_mode = interactive_mode_;
  }

  visualization_msgs::MarkerArray marray;
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "slam_toolbox";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.z = 0.0;
  m.pose.orientation.w = 1.;
  m.scale.x = 0.1; m.scale.y = 0.1; m.scale.z = 0.1;
  m.color.r = 1.0; m.color.g = 0; m.color.b = 0.0; m.color.a = 1.;
  m.action = visualization_msgs::Marker::ADD;
  m.lifetime = ros::Duration(0.);

  uint i=0;
  for (i; i < graph.size(); i++) 
  {
    m.id = i+1;
    m.pose.position.x = graph[i](0);
    m.pose.position.y = graph[i](1);

    if (interactive_mode)
    {
      // marker and metadata
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = "map";
      int_marker.header.stamp = ros::Time::now();
      int_marker.name = std::to_string(i+1);
      int_marker.pose.orientation.w = 1.;
      int_marker.pose.position.x = m.pose.position.x;
      int_marker.pose.position.y = m.pose.position.y;
      int_marker.scale = 0.3;

      // translate control
      visualization_msgs::InteractiveMarkerControl control;
      control.orientation_mode =  visualization_msgs::InteractiveMarkerControl::FIXED;
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
      control_rot.orientation_mode =  visualization_msgs::InteractiveMarkerControl::FIXED;
      control_rot.always_visible = true;
      control_rot.orientation.w = 0;
      control_rot.orientation.x = 0.7071;
      control_rot.orientation.y = 0;
      control_rot.orientation.z = 0.7071;
      control_rot.interaction_mode = \
                       visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back( control_rot );

      interactive_server_->insert(int_marker, \
               boost::bind(&SlamToolbox::ProcessInteractiveFeedback, this, _1));
    }
    else
    {
      marray.markers.push_back(visualization_msgs::Marker(m));
    }
  }

  if(!interactive_mode)
  {
    interactive_server_->clear();
  }

  // if disabled, clears out old markers
  interactive_server_->applyChanges();
  marker_publisher_.publish(marray);
  return;
}

/*****************************************************************************/
void SlamToolbox::ProcessInteractiveFeedback(const \
               visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
/*****************************************************************************/
{
  // was depressed, something moved, and now released
  if (feedback->event_type == \
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP && 
      feedback->mouse_point_valid)
  {
    // we offset by 1
    const int id = std::stoi(feedback->marker_name,nullptr,10) - 1;

    // get yaw
    tfScalar yaw, pitch, roll;
    tf::Quaternion quat(0.,0.,0.,1.0);
    tf::quaternionMsgToTF(feedback->pose.orientation, quat); // relative
    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);

    AddMovedNodes(id, Eigen::Vector3d(feedback->mouse_point.x, \
                  feedback->mouse_point.y, yaw));
  }
  if (feedback->event_type == \
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    // get scan
    const int id = std::stoi(feedback->marker_name,nullptr,10) - 1;
    sensor_msgs::LaserScan scan = current_scans_[id];

    // create correct frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(feedback->pose.position.x, \
                                    feedback->pose.position.y, 0.));

    // get correct orientation
    tf::Quaternion quat(0.,0.,0.,1.0), msg_quat;

    double node_yaw, first_node_yaw;
    solver_->GetNodeOrientation(id, node_yaw);
    solver_->GetNodeOrientation(0, first_node_yaw);

    quat *= tf::Quaternion(0., 0., node_yaw- 3.14159);
    quat *= tf::Quaternion(0., 0., 3.14159); 


    if (lasers_inverted_[scan.header.frame_id])
    {
      sensor_msgs::LaserScan temp;
      for (int i=scan.ranges.size() ;i!=0;i--)
      {
        temp.ranges.push_back(scan.ranges[i]);
        temp.intensities.push_back(scan.intensities[i]);
      }
      scan.ranges = temp.ranges;
      scan.intensities = temp.intensities;
    }

    // interfactive move
    tf::quaternionMsgToTF(feedback->pose.orientation, msg_quat);
    quat *= msg_quat;
    quat.normalize();

    transform.setRotation(quat);
    tfB_->sendTransform(tf::StampedTransform(transform, 
                       ros::Time::now(), "map", "karto_scan_visualization"));
    scan.header.frame_id = "karto_scan_visualization";
    scan.header.stamp = ros::Time::now();
    scan_publisher_.publish(scan);
  }
}

/*****************************************************************************/
void SlamToolbox::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  static karto::Pose2 last_pose;
  static double last_scan_time = 0.;
  static double min_dist2 = minimum_travel_distance_*minimum_travel_distance_;

  // we are in a paused mode, reject incomming information
  if(IsPaused(NEW_MEASUREMENTS))
  {
    return;
  }

  // throttled out
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
  {
    return;
  }

  // not enough time
  if ( (scan->header.stamp.toSec() - last_scan_time ) < minimum_time_interval_)
  {
    return;
  }

  // no odom info
  karto::Pose2 pose;
  if(!GetOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // check moved enough, within 10% for correction error
  const double dist2 = fabs((last_pose.GetX() - pose.GetX())*(last_pose.GetX() - 
                       pose.GetX()) + (last_pose.GetY() - 
                       pose.GetY())*(last_pose.GetX() - pose.GetY()));
  if(dist2 < 0.8*min_dist2 || laser_count_ < 5)
  {
    return;
  }

  // ok... maybe valid we can try
  q_.push(posed_scan(scan, pose));
  last_scan_time = scan->header.stamp.toSec(); 
  last_pose = pose;

  return;
}

/*****************************************************************************/
bool SlamToolbox::UpdateMap()
/*****************************************************************************/
{
  if (!publish_occupancy_map_ || sst_.getNumSubscribers() == 0)
  {
    return true;
  }

  boost::mutex::scoped_lock lock(map_mutex_);

  karto::OccupancyGrid* occ_grid = NULL;
  {
    boost::mutex::scoped_lock lock(mapper_mutex_);
    occ_grid = karto::OccupancyGrid::CreateFromScans( \
                                            mapper_->GetAllProcessedScans(), 
                                                               resolution_);
  }
  
  if(!occ_grid)
  {
    return false;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = \
                            occ_grid->GetCoordinateConverter()->GetOffset();

  if(map_.map.info.width != (unsigned int) width || 
     map_.map.info.height != (unsigned int) height ||
     map_.map.info.origin.position.x != offset.GetX() ||
     map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y=0; y<height; y++)
  {
    for (kt_int32s x=0; x<width; x++) 
    {
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
      switch (value)
      {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }
  
  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
  
  delete occ_grid;
  return true;
}

/*****************************************************************************/
bool SlamToolbox::AddScan(karto::LaserRangeFinder* laser,
		   const sensor_msgs::LaserScan::ConstPtr& scan, 
                   karto::Pose2& karto_pose)
/*****************************************************************************/
{  
  // Create a vector of doubles for karto
  std::vector<kt_double> readings;
  if (lasers_inverted_[scan->header.frame_id]) {
    for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
      it != scan->ranges.rend();
      ++it)
    {
      readings.push_back(*it);
    }
  } else {
    for(std::vector<float>::const_iterator it = scan->ranges.begin();
      it != scan->ranges.end();
      ++it)
    {
      readings.push_back(*it);
    }
  }
  
  // create localized range scan
  karto::LocalizedRangeScan* range_scan = 
    new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  boost::mutex::scoped_lock lock(mapper_mutex_);
  bool processed;
  if((processed = mapper_->Process(range_scan)))
  {
    current_scans_.push_back(*scan);
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> 
        (tf::Transform(tf::createQuaternionFromRPY(0,0,corrected_pose.GetHeading()),
        tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(),
        scan->header.stamp, base_frame_),odom_to_map);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }

    {
      boost::mutex::scoped_lock lock(map_to_odom_mutex_);
      map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                              tf::Point( odom_to_map.getOrigin() ) ).inverse();
    }
    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  }
  else
    delete range_scan;

  return processed;
}

/*****************************************************************************/
void  SlamToolbox::ClearMovedNodes()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(moved_nodes_mutex);
  moved_nodes_.clear();
}

/*****************************************************************************/
void SlamToolbox::AddMovedNodes(const int& id, Eigen::Vector3d vec)
/*****************************************************************************/
{
  ROS_INFO(
    "SlamToolbox: Node %i new manual loop closure pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex);
  moved_nodes_[id] = vec;
}

/*****************************************************************************/
bool SlamToolbox::MapCallback(nav_msgs::GetMap::Request  &req,
                            nav_msgs::GetMap::Response &res)
/*****************************************************************************/
{
  if(map_.map.info.width && map_.map.info.height)
  {
    boost::mutex::scoped_lock lock(map_mutex_);
    res = map_;
    return true;
  }
  else
    return false;
}

/*****************************************************************************/
bool SlamToolbox::ManualLoopClosureCallback( \
                                     slam_toolbox::LoopClosure::Request  &req,
                                     slam_toolbox::LoopClosure::Response &resp)
/*****************************************************************************/
{
  {
    boost::mutex::scoped_lock lock(moved_nodes_mutex);

    if (moved_nodes_.size() == 0)
    {
      ROS_WARN("No moved nodes to attempt manual loop closure.");
      return true;
    }

    ROS_INFO("SlamToolbox: Attempting to manual loop close with %i moved nodes.", 
                                                     (int)moved_nodes_.size());
    // for each in node map
    std::map<int, Eigen::Vector3d>::const_iterator it = moved_nodes_.begin();
    for (it;it!=moved_nodes_.end();++it)
    {
      MoveNode(it->first, \
            Eigen::Vector3d(it->second(0),it->second(1), it->second(2)), false);
    }
  }

  // optimize
  mapper_->CorrectPoses();

  // update visualization and clear out nodes completed
  PublishGraph();  
  ClearMovedNodes();
  return true;
}


/*****************************************************************************/
bool SlamToolbox::InteractiveCallback( \
                               slam_toolbox::ToggleInteractive::Request  &req,
                               slam_toolbox::ToggleInteractive::Response &resp)
/*****************************************************************************/
{
  bool interactive_mode;
  {
    boost::mutex::scoped_lock lock_i(interactive_mutex_);
    interactive_mode_ = !interactive_mode_;   
    interactive_mode = interactive_mode_;
    nh_.setParam("interactive_mode", interactive_mode_);
  }

  ROS_INFO("SlamToolbox: Toggling %s interactive mode.", 
                                             interactive_mode ? "on" : "off");
  PublishGraph();
  ClearMovedNodes();

  boost::mutex::scoped_lock lock_p(pause_mutex_); 
  if (interactive_mode)
  {
    // stop publishing and processing new measurements so we can move things in peace
    pause_graph_  = true;
    pause_processing_ = true;
    nh_.setParam("paused_processing", pause_processing_);
  }
  else
  {
    // exiting interactive mode, continue publishing and processing new measurements
    pause_graph_ = false;
    pause_processing_ = false;
    nh_.setParam("paused_processing", pause_processing_);
  }

  return true;
}


/*****************************************************************************/
bool SlamToolbox::PauseProcessingCallback(slam_toolbox::Pause::Request& req,
                                          slam_toolbox::Pause::Response& resp)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(pause_mutex_); 
  pause_processing_ = !pause_processing_;
  nh_.setParam("paused_processing", pause_processing_);
  ROS_INFO("SlamToolbox: Toggled to %s", \
              pause_processing_ ? "paused processing." : "active processing.");
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SlamToolbox::PauseNewMeasurementsCallback( \
                                          slam_toolbox::Pause::Request& req,
                                          slam_toolbox::Pause::Response& resp)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(pause_mutex_); 
  pause_new_measurements_ = !pause_new_measurements_;
  nh_.setParam("paused_measurements", pause_new_measurements_);
  ROS_INFO("SlamToolbox: Toggled to %s", \
    pause_new_measurements_ ? "pause taking new measurements." : 
    "actively taking new measurements.");
  resp.status = true;
  return true;
}


/*****************************************************************************/
bool SlamToolbox::ClearChangesCallback(slam_toolbox::Clear::Request  &req,
                                       slam_toolbox::Clear::Response &resp)
/*****************************************************************************/
{
  ROS_INFO("SlamToolbox: Clearing manual loop closure nodes.");
  PublishGraph();
  ClearMovedNodes();
  return true;
}

/*****************************************************************************/
bool SlamToolbox::ClearQueueCallback(slam_toolbox::ClearQueue::Request& req,
                                   slam_toolbox::ClearQueue::Response& resp)
/*****************************************************************************/
{
  ROS_INFO("SlamToolbox: Clearing all queued scans to add to map.");
  std::queue<posed_scan> empty;
  std::swap( q_, empty );
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SlamToolbox::SaveMapCallback(slam_toolbox::SaveMap::Request  &req,
                                slam_toolbox::SaveMap::Response &resp)
/*****************************************************************************/
{
  std::vector<Eigen::Vector2d> graph;
  solver_->getGraph(graph);
  if (graph.size() == 0)
  {
    ROS_WARN("Graph is empty, no map to save.");
    return true;
  }

  const std::string name = req.name.data;
  if (name != "")
  {
    ROS_INFO("SlamToolbox: Saving map as %s.", name.c_str());
    system(("rosrun map_server map_saver -f " + name).c_str());
  }
  else
  {
    ROS_INFO("SlamToolbox: Saving map in current directory.");
    system("rosrun map_server map_saver");
  }
  ros::Duration(1.0).sleep();
  return true;
}

/*****************************************************************************/
bool SlamToolbox::IsPaused(const PausedApplication& app)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(pause_mutex_);
  if (app == NEW_MEASUREMENTS)
  {
    return pause_new_measurements_;
  }
  else if (app == PROCESSING)
  {
    return pause_processing_;
  }
  else if (app == VISUALIZING_GRAPH)
  {
    return pause_graph_;
  }
  return false; // then assume working
}

/*****************************************************************************/
void SlamToolbox::Run()
/*****************************************************************************/
{
  ros::Rate r(60);
  while(ros::ok())
  {
    if (!q_.empty() && !IsPaused(PROCESSING))
    {
      posed_scan scan_w_pose = q_.front();
      q_.pop();
      ROS_INFO_THROTTLE(15., "Queue size: %i", (int)q_.size());

      // Check whether we know about this laser yet
      karto::LaserRangeFinder* laser = GetLaser(scan_w_pose.scan);

      if(!laser)
      {
        ROS_WARN("SlamToolbox: Failed to create laser device for %s; discarding scan",
           scan_w_pose.scan->header.frame_id.c_str());
        break;
      }
      AddScan(laser, scan_w_pose.scan, scan_w_pose.pose);
    }
    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::MoveNode(const int& id, const Eigen::Vector3d& pose, \
                           const bool correct)
/*****************************************************************************/
{
  solver_->ModifyNode(id, pose);
  if (correct)
  {
    mapper_->CorrectPoses();
  }
}
/*****************************************************************************/
bool SlamToolbox::SerializePoseGraphCallback(slam_toolbox::SerializePoseGraph::Request  &req,
                                             slam_toolbox::SerializePoseGraph::Response &resp)
/*****************************************************************************/
{
  const std::string filename = req.filename;
  serialization::Write(filename, mapper_, dataset_);
  return true;
}

/*****************************************************************************/
bool SlamToolbox::LoadMapperCallback(slam_toolbox::AddMap::Request  &req,
                                     slam_toolbox::AddMap::Response &resp)
/*****************************************************************************/
{
  // TODO STEVE: this doesnt account for pose changes - need offset value for odometry so frames are colinear
    // ie add field for initial pose of new entries in the frame of the original and a bool whether to continue using the existing odom in TF (when continueing the same session serialized so the odom frame ihasnt changed)
  // TODO STEVE2: how to remove extraneous nodes (?)

  const std::string filename = req.filename;

  karto::Dataset* dataset = new karto::Dataset;
  karto::Mapper* mapper = new karto::Mapper;
  serialization::Read(filename, mapper, dataset);
  std::map<karto::Name, std::vector<karto::Vertex<karto::LocalizedRangeScan>*>> mapper_vertices = mapper->GetGraph()->GetVertices();
  std::map<karto::Name, std::vector<karto::Vertex<karto::LocalizedRangeScan>*>>::iterator vertices_it;
  for(vertices_it = mapper_vertices.begin(); vertices_it!=mapper_vertices.end(); ++vertices_it)
  {
    for(std::vector<karto::Vertex<karto::LocalizedRangeScan>*>::iterator vertex_it=  vertices_it->second.begin();
        vertex_it!= vertices_it->second.end();++vertex_it )
    {
      solver_->AddNode(*vertex_it);
    }
  }
  std::vector<karto::Edge<karto::LocalizedRangeScan>*> mapper_edges = mapper->GetGraph()->GetEdges();
  std::vector<karto::Edge<karto::LocalizedRangeScan>*>::iterator edges_it;
  for( edges_it = mapper_edges.begin(); edges_it != mapper_edges.end(); ++edges_it)
  {
    solver_->AddConstraint(*edges_it);
  }
  mapper->SetScanSolver(solver_.get());
  {
    boost::mutex::scoped_lock lock(mapper_mutex_);
    if (mapper_)
    {
      delete mapper_;
      mapper_ = NULL;
    }
    if (dataset_)
    {
      delete dataset_;
      dataset_ = NULL;
    }

    karto::LaserRangeFinder* laser = dynamic_cast<karto::LaserRangeFinder*>(dataset->GetObjects()[0]);
    mapper_ = mapper;
    dataset_ = dataset;
    karto::Sensor*  pSensor = dynamic_cast<karto::Sensor *>(laser);
    if (pSensor)
    {
      karto::SensorManager::GetInstance()->RegisterSensor(pSensor);
      lasers_[laser_frame_] = laser;
      bool is_inverted;
      if(!nh_.getParam("inverted_laser", is_inverted))
      {
        is_inverted = false;
      }
      lasers_inverted_[laser_frame_] = is_inverted;
    }
  }
  UpdateMap();
}

/*****************************************************************************/
int main(int argc, char** argv)
/*****************************************************************************/
{
  ros::init(argc, argv, "slam_toolbox");
  SlamToolbox kt;
  ros::spin();
  return 0;
}
