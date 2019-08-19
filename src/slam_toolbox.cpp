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

/* Orginal Author for slam_karto: Brian Gerkey */
/* Heavily Modified for slam_toolbox: Steven Macenski */

#include "slam_toolbox/slam_toolbox.hpp"
#include "slam_toolbox/serialization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
SlamToolbox::SlamToolbox()
: solver_loader_("slam_toolbox", "karto::ScanSolver"),
  pause_graph_(false),
  pause_processing_(false),
  pause_new_measurements_(false),
  interactive_mode_(false),
  processor_type_(PROCESS),
  localization_pose_set_(false),
  first_measurement_(true)
/*****************************************************************************/
{
  ros::NodeHandle private_nh("~");
  nh_ = private_nh;

  interactive_server_ =
    std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "slam_toolbox","",true);

  mapper_ = std::make_unique<karto::Mapper>();
  dataset_ = std::make_unique<karto::Dataset>();

  setParams(private_nh);
  setROSInterfaces(private_nh);
  setSolver(private_nh);

  laser_assistant_ = std::make_unique<laser_utils::LaserAssistant>(
    private_nh, tf_.get(), base_frame_);
  pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(
    tf_.get(), base_frame_, odom_frame_);
  current_scans_ = std::make_unique<std::vector<sensor_msgs::LaserScan> >();
  map_saver_ = std::make_unique<map_saver::MapSaver>(nh_, map_name_);

  reprocessing_transform_.setIdentity();

  double transform_publish_period;
  private_nh.param("transform_publish_period", transform_publish_period, 0.05);
  transform_thread_ = std::make_unique<boost::thread>(
    boost::bind(&SlamToolbox::publishTransformLoop,
    this, transform_publish_period));
  run_thread_ = std::make_unique<boost::thread>(
    boost::bind(&SlamToolbox::run, this));
  visualization_thread_ = std::make_unique<boost::thread>(
    boost::bind(&SlamToolbox::publishVisualizations, this));
}

/*****************************************************************************/
SlamToolbox::~SlamToolbox()
/*****************************************************************************/
{
  transform_thread_->join();
  run_thread_->join();
  visualization_thread_->join();
  mapper_.reset();
  dataset_.reset();
}

/*****************************************************************************/
void SlamToolbox::setSolver(ros::NodeHandle& private_nh_)
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
void SlamToolbox::setParams(ros::NodeHandle& private_nh_)
/*****************************************************************************/
{
  map_to_odom_.setIdentity();
  private_nh_.param("online", online_, true);
  private_nh_.param("sychronous", sychronous_, true);
  private_nh_.param("odom_frame", odom_frame_, std::string("odom"));
  private_nh_.param("map_frame", map_frame_, std::string("map"));
  private_nh_.param("base_frame", base_frame_, std::string("base_footprint"));
  private_nh_.param("laser_frame", laser_frame_, std::string("laser_link"));
  private_nh_.param("throttle_scans", throttle_scans_, 1);
  private_nh_.param("publish_occupancy_map", publish_occupancy_map_, false);
  private_nh_.param("resolution", resolution_, 0.05);
  private_nh_.param("map_name", map_name_, std::string("/map"));

  double tmp_val;
  private_nh_.param("transform_timeout", tmp_val, 0.2);
  transform_timeout_ = ros::Duration(tmp_val);
  
  private_nh_.param("tf_buffer_duration", tmp_val, 30.);
  tf_buffer_dur_ = ros::Duration(tmp_val);
  
  private_nh_.param("minimum_time_interval", tmp_val, 0.5);
  minimum_time_interval_ = ros::Duration(tmp_val);

  bool debug = false;
  if (private_nh_.getParam("debug_logging", debug) && debug)
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
      ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();   
    }
  }

  mapper_utils::setMapperParams(private_nh_, mapper_.get());
  minimum_travel_distance_ = mapper_->getParamMinimumTravelDistance();

  nh_.setParam("paused_processing", pause_processing_);
  nh_.setParam("paused_new_measurements", pause_new_measurements_);
  nh_.setParam("interactive_mode", interactive_mode_);
}

/*****************************************************************************/
void SlamToolbox::setROSInterfaces(ros::NodeHandle& node)
/*****************************************************************************/
{
  tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(tf_buffer_dur_));
  tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  sst_ = node.advertise<nav_msgs::OccupancyGrid>(map_name_, 1, true);
  sstm_ = node.advertise<nav_msgs::MapMetaData>(map_name_ + "_metadata", 1, true);
  localization_pose_sub_ = node.subscribe("/initialpose", 2, &SlamToolbox::localizePoseCallback, this);
  ssMap_ = node.advertiseService("dynamic_map", &SlamToolbox::mapCallback, this);
  ssClear_ = node.advertiseService("clear_queue", &SlamToolbox::clearQueueCallback, this);
  ssPause_measurements_ = node.advertiseService("pause_new_measurements", &SlamToolbox::pauseNewMeasurementsCallback, this);
  ssLoopClosure_ = node.advertiseService("manual_loop_closure", &SlamToolbox::manualLoopClosureCallback, this);
  ssInteractive_ = node.advertiseService("toggle_interactive_mode", &SlamToolbox::interactiveCallback,this);
  ssClear_manual_ = node.advertiseService("clear_changes", &SlamToolbox::clearChangesCallback, this);
  ssSerialize_ = node.advertiseService("serialize_map", &SlamToolbox::serializePoseGraphCallback, this);
  ssLoadMap_ = node.advertiseService("deserialize_map", &SlamToolbox::deserializePoseGraphCallback, this);
  scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan> >(node, "/scan", 5);
  scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan> >(*scan_filter_sub_, *tf_, odom_frame_, 5, node);
  scan_filter_->registerCallback(boost::bind(&SlamToolbox::laserCallback, this, _1));
  marker_publisher_ = node.advertise<visualization_msgs::MarkerArray>("karto_graph_visualization",1);
  scan_publisher_ = node.advertise<sensor_msgs::LaserScan>("karto_scan_visualization",10);
}

/*****************************************************************************/
void SlamToolbox::publishTransformLoop(const double& transform_publish_period)
/*****************************************************************************/
{
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok())
  {
    {
      boost::mutex::scoped_lock lock(map_to_odom_mutex_);
      geometry_msgs::TransformStamped msg;
      tf2::convert(map_to_odom_, msg.transform);
      msg.child_frame_id = odom_frame_;
      msg.header.frame_id = map_frame_;
      msg.header.stamp = ros::Time::now() + transform_timeout_;
      tfB_->sendTransform(msg);
    }
    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::publishVisualizations()
/*****************************************************************************/
{
  nav_msgs::OccupancyGrid& og = map_.map;
  og.info.resolution = resolution_;
  og.info.origin.position.x = 0.0;
  og.info.origin.position.y = 0.0;
  og.info.origin.position.z = 0.0;
  og.info.origin.orientation.x = 0.0;
  og.info.origin.orientation.y = 0.0;
  og.info.origin.orientation.z = 0.0;
  og.info.origin.orientation.w = 1.0;
  og.header.frame_id = map_frame_;

  double map_update_interval;
  if(!nh_.getParam("map_update_interval", map_update_interval))
    map_update_interval = 10.0;
  ros::Rate r(1.0 / map_update_interval);

  while(ros::ok())
  {
    updateMap();
    if(!isPaused(VISUALIZING_GRAPH))
    {
      publishGraph();
    }
    r.sleep();
  }
}

/*****************************************************************************/
karto::LaserRangeFinder* SlamToolbox::getLaser(const
  sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  const std::string& frame = scan->header.frame_id;

  if (laser_frame_ != frame)
  {
    ROS_FATAL_ONCE("Laser param frame: %s is not the same as the scan frame: %s."
      " This WILL cause fatal issues with deserialization or lifelong mapping.",
      laser_frame_.c_str(), frame.c_str());
  }

  if(lasers_.find(frame) == lasers_.end())
  {
    try
    {
      lasers_[frame] = laser_assistant_->toLaserMetadata(*scan);
      dataset_->Add(lasers_[frame].getLaser(), true);
    }
    catch (tf2::TransformException& e)
    {
      ROS_ERROR("Failed to compute laser pose, aborting initialization (%s)",
        e.what());
      return nullptr;
    }
  }

  return lasers_[frame].getLaser();
}

/*****************************************************************************/
void SlamToolbox::publishGraph()
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
  visualization_msgs::Marker m = vis_utils::toMarker(map_frame_, "slam_toolbox", 0.1);

  for (const_graph_iterator it = graph->begin(); it != graph->end(); ++it)
  {
    m.id = it->first + 1;
    m.pose.position.x = it->second(0);
    m.pose.position.y = it->second(1);

    if (interactive_mode)
    {
      visualization_msgs::InteractiveMarker int_marker =
        vis_utils::toInteractiveMarker(m, 0.3);
      interactive_server_->insert(int_marker,
        boost::bind(&SlamToolbox::processInteractiveFeedback, this, _1));
    }
    else
    {
      marray.markers.push_back(visualization_msgs::Marker(m));
    }
  }

  // if disabled, clears out old markers
  interactive_server_->applyChanges();
  marker_publisher_.publish(marray);
  return;
}

/*****************************************************************************/
sensor_msgs::LaserScan SlamToolbox::getCorrectedScan(const int& id)
/*****************************************************************************/
{
  sensor_msgs::LaserScan scan = current_scans_->at(id);
  const laser_utils::LaserMetadata& laser = lasers_[scan.header.frame_id];
  if (laser.isInverted())
  {
    laser.invertScan(scan);
  }

  return scan;
}

/*****************************************************************************/
void SlamToolbox::processInteractiveFeedback(const
  visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION)
  {
    ROS_WARN_ONCE("Cannot manually correct nodes in localization modes. "
      "This message will appear once.");
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
    sensor_msgs::LaserScan scan = getCorrectedScan(id);

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
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    tfB_->sendTransform(msg);

    scan.header.frame_id = "karto_scan_visualization";
    scan.header.stamp = ros::Time::now();
    scan_publisher_.publish(scan);
  }
}

/*****************************************************************************/
void SlamToolbox::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  // no odom info
  karto::Pose2 pose;
  if(!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // ensure the laser can be used
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
      scan->header.frame_id.c_str());
    return;
  }

  if (!sychronous_)
  {
    addScan(laser, scan, pose);
    return;
  }

  if (!shouldProcessScan(scan, pose))
  {
    return;
  }

  q_.push(posedScan(scan, pose));
  return;
}

/*****************************************************************************/
bool SlamToolbox::shouldProcessScan(
  const sensor_msgs::LaserScan::ConstPtr& scan,
  const karto::Pose2& pose)
/*****************************************************************************/
{
  static karto::Pose2 last_pose;
  static ros::Time last_scan_time = ros::Time(0.);
  static double min_dist2 = minimum_travel_distance_ * minimum_travel_distance_;

  // we give it a pass on the first measurement to get the ball rolling
  if (first_measurement_)
  {
    last_scan_time = scan->header.stamp;
    last_pose = pose;
    first_measurement_ = false;
    return true;
  }

  // we are in a paused mode, reject incomming information
  if(isPaused(NEW_MEASUREMENTS))
  {
    return false;
  }

  // throttled out
  if ((scan->header.seq % throttle_scans_) != 0)
  {
    return false;
  }

  // not enough time
  if (scan->header.stamp - last_scan_time < minimum_time_interval_)
  {
    return false;
  }

  // check moved enough, within 10% for correction error
  const double dist2 = fabs((last_pose.GetX() - pose.GetX())*(last_pose.GetX() - 
    pose.GetX()) + (last_pose.GetY() - pose.GetY())*
    (last_pose.GetX() - pose.GetY()));
  if(dist2 < 0.8 * min_dist2 || scan->header.seq < 5)
  {
    return false;
  }

  last_pose = pose;
  last_scan_time = scan->header.stamp; 

  return true;
}

/*****************************************************************************/
bool SlamToolbox::updateMap()
/*****************************************************************************/
{
  if (!publish_occupancy_map_ || sst_.getNumSubscribers() == 0)
  {
    return true;
  }

  boost::mutex::scoped_lock lock(map_mutex_);

  karto::OccupancyGrid* occ_grid = nullptr;
  {
    boost::mutex::scoped_lock lock(mapper_mutex_);
    occ_grid = karto::OccupancyGrid::CreateFromScans(
      mapper_->GetAllProcessedScans(), resolution_);
  }
  
  if(!occ_grid)
  {
    ROS_WARN("UpdateMap: Unable to create map from scans.");
    return false;
  }

  vis_utils::toNavMap(occ_grid, map_.map);

  // publish map as current
  map_.map.header.stamp = ros::Time::now();
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
  
  delete occ_grid;
  occ_grid = nullptr;
  return true;
}

/*****************************************************************************/
bool SlamToolbox::addScan(
  karto::LaserRangeFinder* laser,
	const sensor_msgs::LaserScan::ConstPtr& scan, 
  karto::Pose2& karto_pose)
/*****************************************************************************/
{  
  // Create a vector of doubles for karto
  std::vector<kt_double> readings = laser_utils::scanToReadings(
    *scan, lasers_[scan->header.frame_id].isInverted());

  tf2::Transform pose_original = kartoPose2TfPose(karto_pose);
  tf2::Transform tf_pose_transformed = reprocessing_transform_ * pose_original;
  karto::Pose2 transformed_pose = tfPose2KartoPose(tf_pose_transformed);

  // create localized range scan
  karto::LocalizedRangeScan* range_scan = 
    new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(transformed_pose);
  range_scan->SetCorrectedPose(transformed_pose);

  // Add the localized range scan to the mapper
  boost::mutex::scoped_lock lock(mapper_mutex_);
  bool processed = false, update_offset = false;
  bool localize_first_match = PROCESS_LOCALIZATION && !localization_pose_set_;

  if (processor_type_ == PROCESS)
  {
    processed = mapper_->Process(range_scan);
  }
  else if (processor_type_ == PROCESS_FIRST_NODE)
  {
    processed = mapper_->ProcessAtDock(range_scan);
    processor_type_ = PROCESS;
    update_offset = true;
  }
  else if (processor_type_ == PROCESS_NEAR_REGION || localize_first_match)
  {
    karto::Pose2 estimated_starting_pose;
    estimated_starting_pose.SetX(process_near_pose_.x);
    estimated_starting_pose.SetY(process_near_pose_.y);
    estimated_starting_pose.SetHeading(process_near_pose_.theta);
    range_scan->SetOdometricPose(estimated_starting_pose);
    range_scan->SetCorrectedPose(estimated_starting_pose);
    processed = mapper_->ProcessAgainstNodesNearBy(range_scan);
    update_offset = true;
    if (processor_type_ == PROCESS_LOCALIZATION)
    {
      localization_pose_set_ = true;
      processor_type_ = PROCESS_LOCALIZATION;
    }
    else
    {
      processor_type_ = PROCESS;   
    }
  }
  else if (processor_type_ == PROCESS_LOCALIZATION)
  {
    processed = mapper_->ProcessLocalization(range_scan);
  }
  else
  {
    ROS_FATAL("No valid processor type set! Exiting.");
    exit(-1);
  }

  if(processed)
  {
    current_scans_->push_back(*scan);
    const karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
    tf2::Stamped<tf2::Transform> odom_to_map;
    tf2::Quaternion q(0.,0.,0.,1.0);
    q.setRPY(0., 0., corrected_pose.GetHeading());
    tf2::Stamped<tf2::Transform> base_to_map(
                            tf2::Transform(
                              q,
                              tf2::Vector3(corrected_pose.GetX(), 
                                          corrected_pose.GetY(), 
                                          0.0)
                            ).inverse(), 
                            scan->header.stamp, base_frame_);
    try
    {
      geometry_msgs::TransformStamped base_to_map_msg, odom_to_map_msg;
      tf2::convert(base_to_map, base_to_map_msg);
      odom_to_map_msg = tf_->transform(base_to_map_msg, odom_frame_);
      tf2::convert(odom_to_map_msg, odom_to_map);
    }
    catch(tf2::TransformException& e)
    {
      ROS_ERROR("Transform from base_link to odom failed: %s", e.what());
      odom_to_map.setIdentity();
    }

    {
      boost::mutex::scoped_lock lock(map_to_odom_mutex_);
      map_to_odom_ = tf2::Transform(tf2::Quaternion( odom_to_map.getRotation() ),
        tf2::Vector3( odom_to_map.getOrigin() ) ).inverse();
    }

    // if we're continuing a previous session, we need to
    // estimate the homogenous transformation between the old and new
    // odometry frames and transform the new session 
    // into the older session's frame
    if (update_offset)
    {
      tf2::Transform odom_to_base_serialized = base_to_map.inverse();
      tf2::Quaternion q1(0.,0.,0.,1.0);
      q1.setRPY(0., 0., tf2::getYaw(odom_to_base_serialized.getRotation()));
      odom_to_base_serialized.setRotation(q1);
      tf2::Transform odom_to_base_current = kartoPose2TfPose(karto_pose);
      reprocessing_transform_ = odom_to_base_serialized * odom_to_base_current.inverse();
    }

    // Add the localized range scan to the dataset for memory management
    if (processor_type_ != PROCESS_LOCALIZATION)
    {
      dataset_->Add(range_scan);
    }
  }
  else
  {
    delete range_scan;
  }

  return processed;
}

/*****************************************************************************/
void  SlamToolbox::clearMovedNodes()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_.clear();
}

/*****************************************************************************/
void SlamToolbox::addMovedNodes(const int& id, Eigen::Vector3d vec)
/*****************************************************************************/
{
  ROS_INFO(
    "SlamToolbox: Node %i new manual loop closure pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_[id] = vec;
}

/*****************************************************************************/
bool SlamToolbox::mapCallback(
  nav_msgs::GetMap::Request &req,
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
bool SlamToolbox::manualLoopClosureCallback(
  slam_toolbox::LoopClosure::Request  &req,
  slam_toolbox::LoopClosure::Response &resp)
/*****************************************************************************/
{
  {
    boost::mutex::scoped_lock lock(moved_nodes_mutex_);

    if (moved_nodes_.size() == 0)
    {
      ROS_WARN("No moved nodes to attempt manual loop closure.");
      return true;
    }

    ROS_INFO("SlamToolbox: Attempting to manual loop close with %i moved nodes.", 
      (int)moved_nodes_.size());
    // for each in node map
    std::map<int, Eigen::Vector3d>::const_iterator it = moved_nodes_.begin();
    for (it; it != moved_nodes_.end(); ++it)
    {
      moveNode(it->first,
        Eigen::Vector3d(it->second(0),it->second(1), it->second(2)), false);
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
bool SlamToolbox::interactiveCallback(
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
  publishGraph();
  clearMovedNodes();

  boost::mutex::scoped_lock lock_p(pause_mutex_); 
  if (interactive_mode)
  {
    // stop publishing / processing new measurements so we can move things
    pause_graph_  = true;
    pause_processing_ = true;
    nh_.setParam("paused_processing", pause_processing_);
  }
  else
  {
    // exiting interactive mode, continue publishing / processing measurements
    pause_graph_ = false;
    pause_processing_ = false;
    nh_.setParam("paused_processing", pause_processing_);
  }

  return true;
}

/*****************************************************************************/
bool SlamToolbox::pauseNewMeasurementsCallback(
  slam_toolbox::Pause::Request& req,
  slam_toolbox::Pause::Response& resp)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(pause_mutex_); 
  pause_new_measurements_ = !pause_new_measurements_;
  nh_.setParam("paused_new_measurements", pause_new_measurements_);
  ROS_INFO("SlamToolbox: Toggled to %s",
    pause_new_measurements_ ? "pause taking new measurements." : 
    "actively taking new measurements.");
  resp.status = true;
  return true;
}


/*****************************************************************************/
bool SlamToolbox::clearChangesCallback(
  slam_toolbox::Clear::Request  &req,
  slam_toolbox::Clear::Response &resp)
/*****************************************************************************/
{
  ROS_INFO("SlamToolbox: Clearing manual loop closure nodes.");
  publishGraph();
  clearMovedNodes();
  return true;
}

/*****************************************************************************/
bool SlamToolbox::clearQueueCallback(
  slam_toolbox::ClearQueue::Request& req,
  slam_toolbox::ClearQueue::Response& resp)
/*****************************************************************************/
{
  ROS_INFO("SlamToolbox: Clearing all queued scans to add to map.");
  while(!q_.empty())
  {
    q_.pop();
  }
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SlamToolbox::isPaused(const PausedApplication& app)
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
void SlamToolbox::run()
/*****************************************************************************/
{
  if (!sychronous_)
  {
    ROS_DEBUG("Exiting Run thread - asynchronous mode selected.");
    return;
  }

  ROS_INFO("Run thread enabled - synchronous mode selected.");

  ros::Rate r(100);
  while(ros::ok())
  {
    if (!q_.empty() && !isPaused(PROCESSING))
    {
      posedScan scanWithPose = q_.front();
      q_.pop();

      if (q_.size() > 10)
      {
        ROS_WARN_THROTTLE(10., "Queue size has grown to: %i. "
          "Recommend stopping until message is gone if online mapping.",
          (int)q_.size());
      }

      addScan(getLaser(scanWithPose.scan), scanWithPose.scan, scanWithPose.pose);
      continue;
    }

    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::moveNode(
  const int& id, const Eigen::Vector3d& pose, const bool correct)
/*****************************************************************************/
{
  solver_->ModifyNode(id, pose);
  if (correct)
  {
    mapper_->CorrectPoses();
  }
}

/*****************************************************************************/
bool SlamToolbox::serializePoseGraphCallback(
  slam_toolbox::SerializePoseGraph::Request  &req,
  slam_toolbox::SerializePoseGraph::Response &resp)
/*****************************************************************************/
{
  std::string filename = req.filename;

  // if we're inside the snap, we need to write to commonly accessible space
  if (snap_utils::isInSnap())
  {
    filename = snap_utils::getSnapPath() + std::string("/") + filename;
  }

  boost::mutex::scoped_lock lock(mapper_mutex_);
  serialization::write(filename, *mapper_, *dataset_);
  return true;
}

/*****************************************************************************/
void SlamToolbox::loadSerializedPoseGraph(
  std::unique_ptr<karto::Mapper>& mapper,
  std::unique_ptr<karto::Dataset>& dataset)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mapper_mutex_);

  solver_->Reset();

  // add the nodes and constraints to the optimizer
  VerticeMap mapper_vertices = mapper->GetGraph()->GetVertices();
  VerticeMap::iterator vertex_map_it = mapper_vertices.begin();
  for(vertex_map_it; vertex_map_it != mapper_vertices.end(); ++vertex_map_it)
  {
    ScanVector::iterator vertex_it = vertex_map_it->second.begin();
    for(vertex_map_it; vertex_it != vertex_map_it->second.end(); ++vertex_it)
    {
      solver_->AddNode(*vertex_it);
    }
  }

  EdgeVector mapper_edges = mapper->GetGraph()->GetEdges();
  EdgeVector::iterator edges_it = mapper_edges.begin();
  for( edges_it; edges_it != mapper_edges.end(); ++edges_it)
  {
    solver_->AddConstraint(*edges_it);
  }

  mapper->SetScanSolver(solver_.get());

  // move the memory to our working dataset
  mapper_.reset();
  dataset_.reset();
  mapper_.swap(mapper);
  dataset_.swap(dataset);

  if (dataset_->GetObjects().size() < 1)
  {
    ROS_FATAL("DeserializePoseGraph: Cannot deserialize "
      "dataset with no laser objects.");
    exit(-1);
  }

  // create a current laser sensor
  karto::LaserRangeFinder* laser =
    dynamic_cast<karto::LaserRangeFinder*>(dataset_->GetObjects()[0]);
  karto::Sensor* pSensor = dynamic_cast<karto::Sensor *>(laser);
  if (pSensor)
  {
    karto::SensorManager::GetInstance()->RegisterSensor(pSensor);

    while (true)
    {
      ROS_INFO("Waiting for scan to get metadata...");
      boost::shared_ptr<sensor_msgs::LaserScan const> scan =
        ros::topic::waitForMessage<sensor_msgs::LaserScan>(
        std::string("/scan"), ros::Duration(1.0));
      if (scan)
      {
        ROS_INFO("Got scan!");
        lasers_[laser_frame_] = laser_assistant_->toLaserMetadata(*scan);
        break;
      }
    }
  }
  else
  {
    ROS_ERROR("Invalid sensor pointer in dataset. Unable to register sensor.");
  }

  return;
}

/*****************************************************************************/
bool SlamToolbox::deserializePoseGraphCallback(
  slam_toolbox::DeserializePoseGraph::Request  &req,
  slam_toolbox::DeserializePoseGraph::Response &resp)
/*****************************************************************************/
{
  if (req.match_type == slam_toolbox::DeserializePoseGraph::Request::UNSET) 
  {
    ROS_ERROR("Deserialization called without valid processor type set. "
      "Undefined behavior!");
    return false;
  }

  std::string filename = req.filename;

  std::unique_ptr<karto::Dataset> dataset = std::make_unique<karto::Dataset>();
  std::unique_ptr<karto::Mapper> mapper = std::make_unique<karto::Mapper>();
  if (filename.empty())
  {
    ROS_WARN("No map file given!");
    return true;
  }

  // if we're inside the snap, we need to write to commonly accessible space
  if (snap_utils::isInSnap())
  {
    filename = snap_utils::getSnapPath() + std::string("/") + filename;
  }

  if (!serialization::read(filename, *mapper, *dataset))
  {
    ROS_ERROR("DeserializePoseGraph: Failed to read "
      "file: %s.", filename.c_str());
    return true;
  }
  ROS_DEBUG("DeserializePoseGraph: Successfully read file.");

  loadSerializedPoseGraph(mapper, dataset);

  solver_->Compute();
  updateMap();

  first_measurement_ = true;
  switch (req.match_type)
  {
    case procType::START_AT_FIRST_NODE:
      processor_type_ = PROCESS_FIRST_NODE;
      break;
    case procType::START_AT_GIVEN_POSE:
      processor_type_ = PROCESS_NEAR_REGION;
      process_near_pose_ = req.initial_pose;
      break;
    case procType::LOCALIZE_AT_POSE: 
      processor_type_ = PROCESS_LOCALIZATION;
      process_near_pose_ = req.initial_pose;
      localization_pose_set_ = false;
      break;
    default:
      ROS_FATAL("Deserialization called without valid processor type set.");
      exit(-1);
  }

  return true;
}

/*****************************************************************************/
void SlamToolbox::localizePoseCallback(const
  geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION)
  {
    ROS_ERROR("LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  process_near_pose_.x = msg->pose.pose.position.x;
  process_near_pose_.y = msg->pose.pose.position.y;
  process_near_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
  localization_pose_set_ = false;
  first_measurement_ = true;

  ROS_INFO("LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    process_near_pose_.x, process_near_pose_.y,
    process_near_pose_.theta);
  return;
}

} // end namespace
