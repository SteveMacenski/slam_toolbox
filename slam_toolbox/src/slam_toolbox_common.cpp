/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Samsung Research America
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

/* Orginal Author for slam_karto the original work was based on: Brian Gerkey */
/* Author: Steven Macenski */

#include "slam_toolbox/slam_toolbox_common.hpp"
#include "slam_toolbox/serialization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
SlamToolbox::SlamToolbox(ros::NodeHandle& nh)
: solver_loader_("slam_toolbox", "karto::ScanSolver"),
  processor_type_(PROCESS),
  first_measurement_(true),
  nh_(nh),
  process_near_pose_(nullptr)
/*****************************************************************************/
{
  smapper_ = std::make_unique<mapper_utils::SMapper>();
  dataset_ = std::make_unique<karto::Dataset>();

  setParams(nh_);
  setROSInterfaces(nh_);
  setSolver(nh_);

  laser_assistant_ = std::make_unique<laser_utils::LaserAssistant>(
    nh_, tf_.get(), base_frame_);
  pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(
    tf_.get(), base_frame_, odom_frame_);
  scan_holder_ = std::make_unique<laser_utils::ScanHolder>(lasers_);
  map_saver_ = std::make_unique<map_saver::MapSaver>(nh_, map_name_);
  closure_assistant_ =
    std::make_unique<loop_closure_assistant::LoopClosureAssistant>(
    nh_, smapper_->getMapper(), scan_holder_.get(), state_, processor_type_);

  reprocessing_transform_.setIdentity();

  double transform_publish_period;
  nh_.param("transform_publish_period", transform_publish_period, 0.05);
  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&SlamToolbox::publishTransformLoop,
    this, transform_publish_period)));
  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&SlamToolbox::publishVisualizations, this)));
}

/*****************************************************************************/
SlamToolbox::~SlamToolbox()
/*****************************************************************************/
{
  for (int i=0; i != threads_.size(); i++)
  {
    threads_[i]->join();
  }

  smapper_.reset();
  dataset_.reset();
  closure_assistant_.reset();
  map_saver_.reset();
  pose_helper_.reset();
  laser_assistant_.reset();
  scan_holder_.reset();
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
    solver_plugin = "solver_plugins::CeresSolver";
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
  smapper_->getMapper()->SetScanSolver(solver_.get());
}

/*****************************************************************************/
void SlamToolbox::setParams(ros::NodeHandle& private_nh)
/*****************************************************************************/
{
  map_to_odom_.setIdentity();
  private_nh.param("odom_frame", odom_frame_, std::string("odom"));
  private_nh.param("map_frame", map_frame_, std::string("map"));
  private_nh.param("base_frame", base_frame_, std::string("base_footprint"));
  private_nh.param("resolution", resolution_, 0.05);
  private_nh.param("map_name", map_name_, std::string("/map"));
  private_nh.param("scan_topic", scan_topic_, std::string("/scan"));
  private_nh.param("throttle_scans", throttle_scans_, 1);
  private_nh.param("enable_interactive_mode", enable_interactive_mode_, false);

  double tmp_val;
  private_nh.param("transform_timeout", tmp_val, 0.2);
  transform_timeout_ = ros::Duration(tmp_val);
  private_nh.param("tf_buffer_duration", tmp_val, 30.);
  tf_buffer_dur_ = ros::Duration(tmp_val);
  private_nh.param("minimum_time_interval", tmp_val, 0.5);
  minimum_time_interval_ = ros::Duration(tmp_val);

  bool debug = false;
  if (private_nh.getParam("debug_logging", debug) && debug)
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
      ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();   
    }
  }

  smapper_->configure(private_nh);
  private_nh.setParam("paused_new_measurements", false);
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
  ssMap_ = node.advertiseService("dynamic_map", &SlamToolbox::mapCallback, this);
  ssPauseMeasurements_ = node.advertiseService("pause_new_measurements", &SlamToolbox::pauseNewMeasurementsCallback, this);
  ssSerialize_ = node.advertiseService("serialize_map", &SlamToolbox::serializePoseGraphCallback, this);
  ssDesserialize_ = node.advertiseService("deserialize_map", &SlamToolbox::deserializePoseGraphCallback, this);
  scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan> >(node, scan_topic_, 5);
  scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan> >(*scan_filter_sub_, *tf_, odom_frame_, 5, node);
  scan_filter_->registerCallback(boost::bind(&SlamToolbox::laserCallback, this, _1));
}

/*****************************************************************************/
void SlamToolbox::publishTransformLoop(const double& transform_publish_period)
/*****************************************************************************/
{
  if(transform_publish_period == 0)
  {
    return;
  }

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
      closure_assistant_->publishGraph();
    }
    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::loadPoseGraphByParams(ros::NodeHandle& nh)
/*****************************************************************************/
{
  std::string filename;
  geometry_msgs::Pose2D pose;
  bool dock = false;
  if (shouldStartWithPoseGraph(filename, pose, dock))
  {
    slam_toolbox_msgs::DeserializePoseGraph::Request req;
    slam_toolbox_msgs::DeserializePoseGraph::Response resp;
    req.initial_pose = pose;
    req.filename = filename;
    if (dock)
    {
      req.match_type =
        slam_toolbox_msgs::DeserializePoseGraph::Request::START_AT_FIRST_NODE;
    }
    else
    {
      req.match_type =
        slam_toolbox_msgs::DeserializePoseGraph::Request::START_AT_GIVEN_POSE;      
    }
    deserializePoseGraphCallback(req, resp);
  }
}

/*****************************************************************************/
bool SlamToolbox::shouldStartWithPoseGraph(std::string& filename,
  geometry_msgs::Pose2D& pose, bool& start_at_dock)
/*****************************************************************************/
  {
  // if given a map to load at run time, do it.
  if (nh_.getParam("map_file_name", filename))
  {
    std::vector<double> read_pose;
    if (nh_.getParam("map_start_pose", read_pose))
    {
      start_at_dock = false;
      if (read_pose.size() != 3)
      {
        ROS_ERROR("LocalizationSlamToolbox: Incorrect number of "
          "arguments for map starting pose. Must be in format: "
          "[x, y, theta]. Starting at the origin");
        pose.x = 0.;
        pose.y = 0.;
        pose.theta = 0.;
      }
      else
      {
        pose.x = read_pose[0];
        pose.y = read_pose[1];
        pose.theta = read_pose[2];
      }
    }
    else
    {
      nh_.getParam("map_start_at_dock", start_at_dock);
    }

    return true;
  }

  return false;
}

/*****************************************************************************/
karto::LaserRangeFinder* SlamToolbox::getLaser(const
  sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  const std::string& frame = scan->header.frame_id;
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
bool SlamToolbox::updateMap()
/*****************************************************************************/
{
  if (sst_.getNumSubscribers() == 0)
  {
    return true;
  }
  boost::mutex::scoped_lock lock(smapper_mutex_);
  karto::OccupancyGrid* occ_grid = smapper_->getOccupancyGrid(resolution_);
  if(!occ_grid)
  {
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
tf2::Stamped<tf2::Transform> SlamToolbox::setTransformFromPoses(
  const karto::Pose2& corrected_pose,
  const karto::Pose2& karto_pose,
  const ros::Time& t,
  const bool& update_reprocessing_transform)
/*****************************************************************************/
{
  // Compute the map->odom transform
  tf2::Stamped<tf2::Transform> odom_to_map;
  tf2::Quaternion q(0.,0.,0.,1.0);
  q.setRPY(0., 0., corrected_pose.GetHeading());
  tf2::Stamped<tf2::Transform> base_to_map(
    tf2::Transform(q, tf2::Vector3(corrected_pose.GetX(),
    corrected_pose.GetY(), 0.0)).inverse(), t, base_frame_);
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
    return odom_to_map;
  }

  // if we're continuing a previous session, we need to
  // estimate the homogenous transformation between the old and new
  // odometry frames and transform the new session 
  // into the older session's frame
  if (update_reprocessing_transform)
  {
    tf2::Transform odom_to_base_serialized = base_to_map.inverse();
    tf2::Quaternion q1(0.,0.,0.,1.0);
    q1.setRPY(0., 0., tf2::getYaw(odom_to_base_serialized.getRotation()));
    odom_to_base_serialized.setRotation(q1);
    tf2::Transform odom_to_base_current = smapper_->toTfPose(karto_pose);
    reprocessing_transform_ = 
      odom_to_base_serialized * odom_to_base_current.inverse();
  }

  // set map to odom for our transformation thread to publish
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  map_to_odom_ = tf2::Transform(tf2::Quaternion( odom_to_map.getRotation() ),
    tf2::Vector3( odom_to_map.getOrigin() ) ).inverse();

  return odom_to_map;
}

/*****************************************************************************/
karto::LocalizedRangeScan* SlamToolbox::getLocalizedRangeScan(
  karto::LaserRangeFinder* laser,
  const sensor_msgs::LaserScan::ConstPtr& scan,
  karto::Pose2& karto_pose)
/*****************************************************************************/
{
  // Create a vector of doubles for karto
  std::vector<kt_double> readings = laser_utils::scanToReadings(
    *scan, lasers_[scan->header.frame_id].isInverted());

  // transform by the reprocessing transform
  tf2::Transform pose_original = smapper_->toTfPose(karto_pose);
  tf2::Transform tf_pose_transformed = reprocessing_transform_ * pose_original;
  karto::Pose2 transformed_pose = smapper_->toKartoPose(tf_pose_transformed);

  // create localized range scan
  karto::LocalizedRangeScan* range_scan = new karto::LocalizedRangeScan(
    laser->GetName(), readings);
  range_scan->SetOdometricPose(transformed_pose);
  range_scan->SetCorrectedPose(transformed_pose);
  return range_scan;
}

/*****************************************************************************/
bool SlamToolbox::shouldProcessScan(
  const sensor_msgs::LaserScan::ConstPtr& scan,
  const karto::Pose2& pose)
/*****************************************************************************/
{
  static karto::Pose2 last_pose;
  static ros::Time last_scan_time = ros::Time(0.);
  static double min_dist2 =
    smapper_->getMapper()->getParamMinimumTravelDistance() *
    smapper_->getMapper()->getParamMinimumTravelDistance();

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
  const double dist2 = last_pose.SquaredDistance(pose);
  if(dist2 < 0.8 * min_dist2 || scan->header.seq < 5)
  {
    return false;
  }

  last_pose = pose;
  last_scan_time = scan->header.stamp; 

  return true;
}

/*****************************************************************************/
karto::LocalizedRangeScan* SlamToolbox::addScan(
  karto::LaserRangeFinder* laser,
  PosedScan& scan_w_pose)
/*****************************************************************************/
{
  return addScan(laser, scan_w_pose.scan, scan_w_pose.pose);
}

/*****************************************************************************/
karto::LocalizedRangeScan* SlamToolbox::addScan(
  karto::LaserRangeFinder* laser,
  const sensor_msgs::LaserScan::ConstPtr& scan, 
  karto::Pose2& karto_pose)
/*****************************************************************************/
{  
  // get our localized range scan
  karto::LocalizedRangeScan* range_scan = getLocalizedRangeScan(
    laser, scan, karto_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;

  if (processor_type_ == PROCESS)
  {
    processed = smapper_->getMapper()->Process(range_scan);
  }
  else if (processor_type_ == PROCESS_FIRST_NODE)
  {
    processed = smapper_->getMapper()->ProcessAtDock(range_scan);
    processor_type_ = PROCESS;
    update_reprocessing_transform = true;
  }
  else if (processor_type_ == PROCESS_NEAR_REGION)
  {
    boost::mutex::scoped_lock l(pose_mutex_);
    if (!process_near_pose_)
    {
      ROS_ERROR("Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan);
    update_reprocessing_transform = true;
    processor_type_ = PROCESS;
  }
  else
  {
    ROS_FATAL("SlamToolbox: No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  // and add our scan to storage
  if(processed)
  {
    if (enable_interactive_mode_)
    {
      scan_holder_->addScan(*scan);
    }

    setTransformFromPoses(range_scan->GetCorrectedPose(), karto_pose,
      scan->header.stamp, update_reprocessing_transform);
    dataset_->Add(range_scan);
  }
  else
  {
    delete range_scan;
    range_scan = nullptr;
  }

  return range_scan;
}

/*****************************************************************************/
bool SlamToolbox::mapCallback(
  nav_msgs::GetMap::Request &req,
  nav_msgs::GetMap::Response &res)
/*****************************************************************************/
{
  if(map_.map.info.width && map_.map.info.height)
  {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    res = map_;
    return true;
  }
  else
  {
    return false;
  }
}

/*****************************************************************************/
bool SlamToolbox::pauseNewMeasurementsCallback(
  slam_toolbox_msgs::Pause::Request& req,
  slam_toolbox_msgs::Pause::Response& resp)
/*****************************************************************************/
{
  bool curr_state = isPaused(NEW_MEASUREMENTS);
  state_.set(NEW_MEASUREMENTS, !curr_state);

  nh_.setParam("paused_new_measurements", !curr_state);
  ROS_INFO("SlamToolbox: Toggled to %s",
    !curr_state ? "pause taking new measurements." : 
    "actively taking new measurements.");
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SlamToolbox::isPaused(const PausedApplication& app)
/*****************************************************************************/
{
  return state_.get(app);
}

/*****************************************************************************/
bool SlamToolbox::serializePoseGraphCallback(
  slam_toolbox_msgs::SerializePoseGraph::Request  &req,
  slam_toolbox_msgs::SerializePoseGraph::Response &resp)
/*****************************************************************************/
{
  std::string filename = req.filename;

  // if we're inside the snap, we need to write to commonly accessible space
  if (snap_utils::isInSnap())
  {
    filename = snap_utils::getSnapPath() + std::string("/") + filename;
  }

  boost::mutex::scoped_lock lock(smapper_mutex_);
  serialization::write(filename, *smapper_->getMapper(), *dataset_);
  return true;
}

/*****************************************************************************/
void SlamToolbox::loadSerializedPoseGraph(
  std::unique_ptr<karto::Mapper>& mapper,
  std::unique_ptr<karto::Dataset>& dataset)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(smapper_mutex_);

  solver_->Reset();

  // add the nodes and constraints to the optimizer
  VerticeMap mapper_vertices = mapper->GetGraph()->GetVertices();
  VerticeMap::iterator vertex_map_it = mapper_vertices.begin();
  for(vertex_map_it; vertex_map_it != mapper_vertices.end(); ++vertex_map_it)
  {
    ScanMap::iterator vertex_it = vertex_map_it->second.begin();
    for(vertex_it; vertex_it != vertex_map_it->second.end(); ++vertex_it)
    {
      if (vertex_it->second != nullptr)
      {
        solver_->AddNode(vertex_it->second);
      }
    }
  }

  EdgeVector mapper_edges = mapper->GetGraph()->GetEdges();
  EdgeVector::iterator edges_it = mapper_edges.begin();
  for( edges_it; edges_it != mapper_edges.end(); ++edges_it)
  {
    if (*edges_it != nullptr)
    {
      solver_->AddConstraint(*edges_it);  
    }
  }

  mapper->SetScanSolver(solver_.get());

  // move the memory to our working dataset
  smapper_->setMapper(mapper.release());
  smapper_->configure(nh_);
  dataset_.reset(dataset.release());

  if (!smapper_->getMapper())
  {
    ROS_FATAL("loadSerializedPoseGraph: Could not properly load "
      "a valid mapping object. Did you modify something by hand?");
    exit(-1);
  }

  if (dataset_->GetLasers().size() < 1)
  {
    ROS_FATAL("loadSerializedPoseGraph: Cannot deserialize "
      "dataset with no laser objects.");
    exit(-1);
  }

  // create a current laser sensor
  karto::LaserRangeFinder* laser =
    dynamic_cast<karto::LaserRangeFinder*>(
    dataset_->GetLasers()[0]);
  karto::Sensor* pSensor = dynamic_cast<karto::Sensor*>(laser);
  if (pSensor)
  {
    karto::SensorManager::GetInstance()->RegisterSensor(pSensor);

    while (ros::ok())
    {
      ROS_INFO("Waiting for incoming scan to get metadata...");
      boost::shared_ptr<sensor_msgs::LaserScan const> scan =
        ros::topic::waitForMessage<sensor_msgs::LaserScan>(
        scan_topic_, ros::Duration(1.0));
      if (scan)
      {
        ROS_INFO("Got scan!");
        try
        {
          lasers_[scan->header.frame_id] =
            laser_assistant_->toLaserMetadata(*scan);
          break;
        }
        catch (tf2::TransformException& e)
        {
          ROS_ERROR("Failed to compute laser pose, aborting continue mapping (%s)",
            e.what());
          exit(-1);
        }
      }
    }
  }
  else
  {
    ROS_ERROR("Invalid sensor pointer in dataset. Unable to register sensor.");
  }

  solver_->Compute();

  return;
}

/*****************************************************************************/
bool SlamToolbox::deserializePoseGraphCallback(
  slam_toolbox_msgs::DeserializePoseGraph::Request  &req,
  slam_toolbox_msgs::DeserializePoseGraph::Response &resp)
/*****************************************************************************/
{
  if (req.match_type == slam_toolbox_msgs::DeserializePoseGraph::Request::UNSET) 
  {
    ROS_ERROR("Deserialization called without valid processor type set. "
      "Undefined behavior!");
    return false;
  }

  std::string filename = req.filename;

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

  std::unique_ptr<karto::Dataset> dataset = std::make_unique<karto::Dataset>();
  std::unique_ptr<karto::Mapper> mapper = std::make_unique<karto::Mapper>();

  if (!serialization::read(filename, *mapper, *dataset))
  {
    ROS_ERROR("DeserializePoseGraph: Failed to read "
      "file: %s.", filename.c_str());
    return true;
  }
  ROS_DEBUG("DeserializePoseGraph: Successfully read file.");

  loadSerializedPoseGraph(mapper, dataset);
  updateMap();

  first_measurement_ = true;
  boost::mutex::scoped_lock l(pose_mutex_);
  switch (req.match_type)
  {
    case procType::START_AT_FIRST_NODE:
      processor_type_ = PROCESS_FIRST_NODE;
      break;
    case procType::START_AT_GIVEN_POSE:
      processor_type_ = PROCESS_NEAR_REGION;
      process_near_pose_ = std::make_unique<karto::Pose2>(req.initial_pose.x, 
        req.initial_pose.y, req.initial_pose.theta);
      break;
    case procType::LOCALIZE_AT_POSE: 
      processor_type_ = PROCESS_LOCALIZATION;
      process_near_pose_ = std::make_unique<karto::Pose2>(req.initial_pose.x, 
        req.initial_pose.y, req.initial_pose.theta);
      break;
    default:
      ROS_FATAL("Deserialization called without valid processor type set.");
  }

  return true;
}

} // end namespace
