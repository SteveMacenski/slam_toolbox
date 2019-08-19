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
  transform_timeout_(ros::Duration(0.2)),
  processor_type_(PROCESS),
  localization_pose_set_(false),
  first_measurement_(true),
  map_name_("/map")
/*****************************************************************************/
{
  ros::NodeHandle private_nh("~");
  nh_ = private_nh;

  interactive_server_ =
    std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "slam_toolbox","",true);

  mapper_ = std::make_unique<karto::Mapper>();
  dataset_ = std::make_unique<karto::Dataset>();

  SetParams(private_nh);
  SetROSInterfaces(private_nh);
  SetSolver(private_nh);

  laser_assistant_ = std::make_unique<laser_utils::LaserAssistant>(private_nh, tf_.get(), base_frame_);
  pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(tf_.get(), base_frame_, odom_frame_);
  current_scans_ = std::make_unique<std::vector<sensor_msgs::LaserScan> >();
  map_saver_ = std::make_unique<map_saver::MapSaver>(nh_, map_name_);

  reprocessing_transform_.setIdentity();

  double transform_publish_period;
  private_nh.param("transform_publish_period", transform_publish_period, 0.05);
  transform_thread_ = std::make_unique<boost::thread>(
    boost::bind(&SlamToolbox::PublishTransformLoop,
    this, transform_publish_period));
  run_thread_ = std::make_unique<boost::thread>(
    boost::bind(&SlamToolbox::Run, this));
  visualization_thread_ = std::make_unique<boost::thread>(
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

  if(!private_nh_.getParam("online", online_))
    online_ = true;
  if(!private_nh_.getParam("sychronous", sychronous_))
    sychronous_ = true;
  double timeout;
  if(!private_nh_.getParam("transform_timeout", timeout))
  {
    transform_timeout_ = ros::Duration(0.2);
  }
  else
  {
    transform_timeout_ = ros::Duration(timeout);
  }
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_footprint";
  if(!private_nh_.getParam("laser_frame", laser_frame_))
    laser_frame_ = "laser_link";
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("publish_occupancy_map", publish_occupancy_map_))
    publish_occupancy_map_ = false;
  if(!private_nh_.getParam("minimum_time_interval", minimum_time_interval_))
    minimum_time_interval_ = 0.5;
  if(!private_nh_.getParam("resolution", resolution_))
  {
    resolution_ = 0.05;
  }

  bool debug = false;
  private_nh_.getParam("debug_logging", debug);
  if (debug)
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
void SlamToolbox::SetROSInterfaces(ros::NodeHandle& node)
/*****************************************************************************/
{
  tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(14400.));
  tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  sst_ = node.advertise<nav_msgs::OccupancyGrid>(map_name_, 1, true);
  sstm_ = node.advertise<nav_msgs::MapMetaData>(map_name_ + "_metadata", 1, true);
  localization_pose_sub_ = node.subscribe("/initialpose", 2, &SlamToolbox::LocalizePoseCallback, this);
  ssMap_ = node.advertiseService("dynamic_map", &SlamToolbox::MapCallback, this);
  ssClear_ = node.advertiseService("clear_queue", &SlamToolbox::ClearQueueCallback, this);
  ssPause_measurements_ = node.advertiseService("pause_new_measurements", &SlamToolbox::PauseNewMeasurementsCallback, this);
  ssLoopClosure_ = node.advertiseService("manual_loop_closure", &SlamToolbox::ManualLoopClosureCallback, this);
  ssInteractive_ = node.advertiseService("toggle_interactive_mode", &SlamToolbox::InteractiveCallback,this);
  ssClear_manual_ = node.advertiseService("clear_changes", &SlamToolbox::ClearChangesCallback, this);
  ssSerialize_ = node.advertiseService("serialize_map", &SlamToolbox::SerializePoseGraphCallback, this);
  ssLoadMap_ = node.advertiseService("deserialize_map", &SlamToolbox::DeserializePoseGraphCallback, this);
  scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan> >(node, "/scan", 5);
  scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan> >(*scan_filter_sub_, *tf_, odom_frame_, 5, node);
  scan_filter_->registerCallback(boost::bind(&SlamToolbox::LaserCallback, this, _1));
  marker_publisher_ = node.advertise<visualization_msgs::MarkerArray>("karto_graph_visualization",1);
  scan_publisher_ = node.advertise<sensor_msgs::LaserScan>("karto_scan_visualization",10);
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
void SlamToolbox::PublishTransformLoop(const double& transform_publish_period)
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
      tf2::convert(map_to_odom_, msg.transform); //TODO too much work
      msg.child_frame_id = map_frame_;
      msg.header.frame_id = odom_frame_;
      msg.header.stamp = ros::Time::now() + transform_timeout_;
      tfB_->sendTransform(msg);
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

  double map_update_interval;
  if(!nh_.getParam("map_update_interval", map_update_interval))
    map_update_interval = 10.0;
  ros::Rate r(1.0 / map_update_interval);

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
karto::LaserRangeFinder* SlamToolbox::GetLaser(const
  sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  const std::string & frame = scan->header.frame_id;

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
void SlamToolbox::PublishGraph()
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
  visualization_msgs::Marker m;
  m.header.frame_id = map_frame_;
  m.header.stamp = ros::Time::now();
  m.ns = "slam_toolbox";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.z = 0.0;
  m.pose.orientation.w = 1.;
  m.scale.x = 0.1; m.scale.y = 0.1; m.scale.z = 0.1;
  m.color.r = 1.0; m.color.g = 0; m.color.b = 0.0; m.color.a = 1.;
  m.action = visualization_msgs::Marker::ADD;
  m.lifetime = ros::Duration(0.);

  for (const_graph_iterator it = graph->begin(); it != graph->end(); ++it)
  {
    m.id = it->first + 1;
    m.pose.position.x = it->second(0);
    m.pose.position.y = it->second(1);

    if (interactive_mode)
    {
      // marker and metadata
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = map_frame_;
      int_marker.header.stamp = ros::Time::now();
      int_marker.name = std::to_string(m.id);
      int_marker.pose.orientation.w = 1.;
      int_marker.pose.position.x = m.pose.position.x;
      int_marker.pose.position.y = m.pose.position.y;
      int_marker.scale = 0.3;

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
void SlamToolbox::ProcessInteractiveFeedback(const
  visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION)
  {
    ROS_WARN_ONCE("Cannot manually correct nodes in localization modes. "
      "This message will appear once.");
    return;
  }

  // was depressed, something moved, and now released
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP && 
      feedback->mouse_point_valid)
  {
    // we offset by 1
    const int id = std::stoi(feedback->marker_name,nullptr,10) - 1;

    // get yaw
    tf2::Quaternion quat(0.,0.,0.,1.0);
    tf2::convert(feedback->pose.orientation, quat); // relative

    AddMovedNodes(id, Eigen::Vector3d(feedback->mouse_point.x,
      feedback->mouse_point.y, tf2::getYaw(quat)));
  }

  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    // get scan
    const int id = std::stoi(feedback->marker_name,nullptr,10) - 1;
    sensor_msgs::LaserScan scan = current_scans_->at(id);
    const laser_utils::LaserMetadata& laser = lasers_[scan.header.frame_id];
    if (laser.isInverted())
    {
      laser.invertScan(scan);
    }

    // create correct frame
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(feedback->pose.position.x,
      feedback->pose.position.y, 0.));

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

    // interfactive move
    tf2::convert(feedback->pose.orientation, msg_quat);
    quat *= msg_quat;
    quat.normalize();
    transform.setRotation(quat);

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
void SlamToolbox::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  // no odom info
  karto::Pose2 pose;
  if(!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  if (!sychronous_)
  {
    // asynchonous
    karto::LaserRangeFinder* laser = GetLaser(scan);

    if(!laser)
    {
      ROS_WARN("Failed to create laser device for %s; discarding scan",
        scan->header.frame_id.c_str());
      return;
    }

    AddScan(laser, scan, pose);
    return;
  }

  static karto::Pose2 last_pose;
  static double last_scan_time = 0.;
  static double min_dist2 = minimum_travel_distance_ * minimum_travel_distance_;

  // we are in a paused mode, reject incomming information
  if(IsPaused(NEW_MEASUREMENTS))
  {
    return;
  }

  // let the first measurement pass to start the ball rolling
  if (first_measurement_)
  {
    q_.push(posedScan(scan, pose));
    last_scan_time = scan->header.stamp.toSec(); 
    last_pose = pose;
    first_measurement_ = false;
    return;
  }

  // throttled out
  if ((scan->header.seq % throttle_scans_) != 0)
  {
    return;
  }

  // not enough time
  if ((scan->header.stamp.toSec()-last_scan_time ) < minimum_time_interval_)
  {
    return;
  }

  // check moved enough, within 10% for correction error
  const double dist2 = fabs((last_pose.GetX() - pose.GetX())*(last_pose.GetX() - 
    pose.GetX()) + (last_pose.GetY() - pose.GetY())*
    (last_pose.GetX() - pose.GetY()));
  if(dist2 < 0.8 * min_dist2 || scan->header.seq < 5)
  {
    return;
  }

  q_.push(posedScan(scan, pose));
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
    occ_grid = karto::OccupancyGrid::CreateFromScans(
      mapper_->GetAllProcessedScans(), resolution_);
  }
  
  if(!occ_grid)
  {
    return false;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
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

  for (kt_int32s y = 0; y < height; y++)
  {
    for (kt_int32s x = 0; x < width; x++) 
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
bool SlamToolbox::AddScan(
  karto::LaserRangeFinder* laser,
	const sensor_msgs::LaserScan::ConstPtr& scan, 
  karto::Pose2& karto_pose)
/*****************************************************************************/
{  
  // Create a vector of doubles for karto
  std::vector<kt_double> readings = laser_utils::scanToReadings(*scan, lasers_[scan->header.frame_id].isInverted());

  tf2::Pose pose_original = kartoPose2TfPose(karto_pose);
  tf2::Pose tf_pose_transformed = reprocessing_transform_ * pose_original;

  karto::Pose2 transformed_pose;
  transformed_pose.SetX(tf_pose_transformed.getOrigin().x());
  transformed_pose.SetY(tf_pose_transformed.getOrigin().y());
  transformed_pose.SetHeading(tf2::getYaw(tf_pose_transformed.getRotation()));

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
    estimated_starting_pose.SetX(process_near_region_pose_.x);
    estimated_starting_pose.SetY(process_near_region_pose_.y);
    estimated_starting_pose.SetHeading(process_near_region_pose_.theta);
    range_scan->SetOdometricPose(estimated_starting_pose);
    range_scan->SetCorrectedPose(estimated_starting_pose);
    processed = mapper_->ProcessAgainstNodesNearBy(range_scan);
    process_near_region_pose_ = geometry_msgs::Pose2D();
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
    tf2::Stamped<tf2::Pose> odom_to_map;
    tf2::Quaternion q(0.,0.,0.,1.0);
    q.setRPY(0., 0., corrected_pose.GetHeading());
    tf2::Stamped<tf2::Pose> base_to_map(
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
      tf2::convert(odom_to_map_msg, odom_to_map);//TODO fix all these conversions

      // if we're continuing a previous session, we need to
      // estimate the homogenous transformation between the old and new
      // odometry frames and transform the new session 
      // into the older session's frame
      if (update_offset)
      {
        tf2::Pose odom_to_base_serialized = base_to_map.inverse();
        tf2::Quaternion q1(0.,0.,0.,1.0);
        q1.setRPY(0., 0., tf2::getYaw(odom_to_base_serialized.getRotation()));
        odom_to_base_serialized.setRotation(q1);
        tf2::Pose odom_to_base_current = kartoPose2TfPose(karto_pose);
        reprocessing_transform_ = odom_to_base_serialized * odom_to_base_current.inverse();
      }
    }
    catch(tf2::TransformException& e)
    {
      ROS_ERROR("Transform from base_link to odom failed: %s", e.what());
      odom_to_map.setIdentity();
    }

    {
      boost::mutex::scoped_lock lock(map_to_odom_mutex_);
      map_to_odom_ = tf2::Transform(tf2::Quaternion( odom_to_map.getRotation() ),
        tf2::Point( odom_to_map.getOrigin() ) ).inverse();
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
void  SlamToolbox::ClearMovedNodes()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_.clear();
}

/*****************************************************************************/
void SlamToolbox::AddMovedNodes(const int& id, Eigen::Vector3d vec)
/*****************************************************************************/
{
  ROS_INFO(
    "SlamToolbox: Node %i new manual loop closure pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_[id] = vec;
}

/*****************************************************************************/
bool SlamToolbox::MapCallback(
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
bool SlamToolbox::ManualLoopClosureCallback(
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
      MoveNode(it->first,
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
bool SlamToolbox::InteractiveCallback(
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
bool SlamToolbox::PauseNewMeasurementsCallback(
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
bool SlamToolbox::ClearChangesCallback(
  slam_toolbox::Clear::Request  &req,
  slam_toolbox::Clear::Response &resp)
/*****************************************************************************/
{
  ROS_INFO("SlamToolbox: Clearing manual loop closure nodes.");
  PublishGraph();
  ClearMovedNodes();
  return true;
}

/*****************************************************************************/
bool SlamToolbox::ClearQueueCallback(
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
  if (!sychronous_)
  {
    // asychronous - don't need to run to dequeue
    ROS_INFO("Exiting Run thread - asynchronous mode selected.");
    return;
  }

  ROS_INFO("Run thread enabled - synchronous mode selected.");

  ros::Rate r(100);
  while(ros::ok())
  {
    if (!q_.empty() && !IsPaused(PROCESSING))
    {
      posedScan scanWithPose = q_.front();
      q_.pop();

      if (q_.size() > 3)
      {
        if (online_ && sychronous_)
        {
          ROS_WARN("Queue size has grown to: %i. "
            "Recommend stopping until message is gone.", (int)q_.size());
        }
        else
        {
          ROS_WARN_THROTTLE(10., "Queue size: %i", (int)q_.size());
        }
      }

      // Check whether we know about this laser yet
      karto::LaserRangeFinder* laser = GetLaser(scanWithPose.scan);

      if(!laser)
      {
        ROS_WARN("SlamToolbox: Failed to create laser"
          " device for %s; discarding scan",
          scanWithPose.scan->header.frame_id.c_str());
        break;
      }

      AddScan(laser, scanWithPose.scan, scanWithPose.pose);
      continue;
    }

    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::MoveNode(
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
bool SlamToolbox::SerializePoseGraphCallback(
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
  serialization::Write(filename, *mapper_, *dataset_);
  return true;
}

/*****************************************************************************/
bool SlamToolbox::DeserializePoseGraphCallback(
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

  if (!serialization::Read(filename, *mapper, *dataset))
  {
    ROS_ERROR("DeserializePoseGraph: Failed to read "
      "file: %s.", filename.c_str());
    return true;
  }

  ROS_INFO("DeserializePoseGraph: Successfully read file.");

  {
    boost::mutex::scoped_lock lock(mapper_mutex_);

    solver_->Reset();

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

    mapper_.reset();
    dataset_.reset();
    mapper_.swap(mapper);
    dataset_.swap(dataset);

    if (dataset_->GetObjects().size() < 1)
    {
      ROS_FATAL("DeserializePoseGraph: Cannot deserialize dataset with no laser objects.");
      exit(-1);
    }

    karto::LaserRangeFinder* laser =
      dynamic_cast<karto::LaserRangeFinder*>(dataset_->GetObjects()[0]);
    karto::Sensor* pSensor = dynamic_cast<karto::Sensor *>(laser);
    if (pSensor)
    {
      karto::SensorManager::GetInstance()->RegisterSensor(pSensor);

      while (true)
      {
        ROS_INFO("Waiting for scan to get metadata...");
        boost::shared_ptr<sensor_msgs::LaserScan const> scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(std::string("/scan"), ros::Duration(1.0));
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
      ROS_ERROR("Invalid sensor pointer in dataset. Not able to register sensor.");
    }
  }

  solver_->Compute();
  UpdateMap();

  first_measurement_ = true;
  switch (req.match_type)
  {
    case procType::START_AT_FIRST_NODE:
      processor_type_ = PROCESS_FIRST_NODE;
      break;
    case procType::START_AT_GIVEN_POSE:
      processor_type_ = PROCESS_NEAR_REGION;
      process_near_region_pose_ = req.initial_pose;
      break;
    case procType::LOCALIZE_AT_POSE: 
      processor_type_ = PROCESS_LOCALIZATION;
      process_near_region_pose_ = req.initial_pose;
      localization_pose_set_ = false;
      break;
    default:
      ROS_FATAL("Deserialization called without valid processor type set.");
      exit(-1);
  }

  return true;
}

/*****************************************************************************/
void SlamToolbox::LocalizePoseCallback(const
  geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION)
  {
    ROS_ERROR("LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  process_near_region_pose_.x = msg->pose.pose.position.x;
  process_near_region_pose_.y = msg->pose.pose.position.y;
  process_near_region_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
  localization_pose_set_ = false;
  first_measurement_ = true;

  ROS_INFO("LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    process_near_region_pose_.x, process_near_region_pose_.y,
    process_near_region_pose_.theta);
  return;
}

} // end namespace
