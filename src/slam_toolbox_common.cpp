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

/* Author: Steven Macenski */
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "slam_toolbox/serialization.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace slam_toolbox
{

/*****************************************************************************/
SlamToolbox::SlamToolbox() : SlamToolbox(rclcpp::NodeOptions())
/*****************************************************************************/
{
}

/*****************************************************************************/
SlamToolbox::SlamToolbox(rclcpp::NodeOptions options)
  : Node("slam_toolbox", "", options), solver_loader_("slam_toolbox", "karto::ScanSolver"),
    processor_type_(PROCESS), first_measurement_(true), process_near_pose_(nullptr),
    transform_timeout_(rclcpp::Duration::from_seconds(0.5)),
    minimum_time_interval_(std::chrono::nanoseconds(0)),
    maximum_match_interval_(rclcpp::Duration::from_seconds(-1.0)), slam_running_(true)
/*****************************************************************************/
{
  smapper_ = std::make_unique<mapper_utils::SMapper>();
  dataset_ = std::make_unique<Dataset>();
}

/*****************************************************************************/
void SlamToolbox::configure()
/*****************************************************************************/
{
  setParams();
  setROSInterfaces();
  setSolver();

  // pause new scan processing
  if (!isPaused(NEW_MEASUREMENTS) && paused_at_startup_)
  {
    toggleScanProcessing();
    slam_running_ = false;
  }

  laser_assistant_ =
    std::make_unique<laser_utils::LaserAssistant>(shared_from_this(), tf_.get(), base_frame_);
  pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(tf_.get(), base_frame_, odom_frame_);
  scan_holder_ = std::make_unique<laser_utils::ScanHolder>(lasers_);
  map_saver_   = std::make_unique<map_saver::MapSaver>(shared_from_this(), map_name_);
  closure_assistant_ = std::make_unique<loop_closure_assistant::LoopClosureAssistant>(
    shared_from_this(), smapper_->getMapper(), scan_holder_.get(), state_, processor_type_);
  reprocessing_transform_.setIdentity();

  double transform_publish_period = 0.05;
  transform_publish_period =
    this->declare_parameter("transform_publish_period", transform_publish_period);
  if (transform_publish_period > 0)
  {
    threads_.push_back(std::make_unique<boost::thread>(
      boost::bind(&SlamToolbox::publishTransformLoop, this, transform_publish_period)));
  }
  threads_.push_back(
    std::make_unique<boost::thread>(boost::bind(&SlamToolbox::publishVisualizations, this)));
}

/*****************************************************************************/
SlamToolbox::~SlamToolbox()
/*****************************************************************************/
{
  for (int i = 0; i != threads_.size(); i++)
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
  solver_.reset();
}

/*****************************************************************************/
void SlamToolbox::setSolver()
/*****************************************************************************/
{
  // Set solver to be used in loop closure
  std::string solver_plugin = std::string("solver_plugins::CeresSolver");
  solver_plugin             = this->declare_parameter("solver_plugin", solver_plugin);

  try
  {
    solver_ = solver_loader_.createSharedInstance(solver_plugin);
    RCLCPP_INFO(get_logger(), "Using solver plugin %s", solver_plugin.c_str());
    solver_->Configure(shared_from_this());
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(get_logger(),
                 "Failed to create %s, is it "
                 "registered and built? Exception: %s.",
                 solver_plugin.c_str(), ex.what());
    exit(1);
  }
  smapper_->getMapper()->SetScanSolver(solver_.get());
}

/*****************************************************************************/
void SlamToolbox::setParams()
/*****************************************************************************/
{
  map_to_odom_.setIdentity();
  odom_frame_ = std::string("odom");
  odom_frame_ = this->declare_parameter("odom_frame", odom_frame_);

  map_frame_ = std::string("map");
  map_frame_ = this->declare_parameter("map_frame", map_frame_);

  base_frame_ = std::string("base_footprint");
  base_frame_ = this->declare_parameter("base_frame", base_frame_);

  resolution_ = 0.05;
  resolution_ = this->declare_parameter("resolution", resolution_);

  map_name_ = std::string("/map");
  map_name_ = this->declare_parameter("map_name", map_name_);

  scan_topic_ = std::string("/scan");
  scan_topic_ = this->declare_parameter("scan_topic", scan_topic_);

  throttle_scans_ = 1;
  throttle_scans_ = this->declare_parameter("throttle_scans", throttle_scans_);

  position_covariance_scale_ = 1.0;
  position_covariance_scale_ =
    this->declare_parameter("position_covariance_scale", position_covariance_scale_);

  yaw_covariance_scale_ = 1.0;
  yaw_covariance_scale_ = this->declare_parameter("yaw_covariance_scale", yaw_covariance_scale_);

  double tmp_val          = 0.5;
  tmp_val                 = this->declare_parameter("transform_timeout", tmp_val);
  transform_timeout_      = rclcpp::Duration::from_seconds(tmp_val);
  tmp_val                 = this->declare_parameter("minimum_time_interval", tmp_val);
  minimum_time_interval_  = rclcpp::Duration::from_seconds(tmp_val);
  tmp_val                 = this->declare_parameter("maximum_match_interval", -1.0);
  maximum_match_interval_ = rclcpp::Duration::from_seconds(tmp_val);

  bool debug = false;
  debug      = this->declare_parameter("debug_logging", debug);
  if (debug)
  {
    rcutils_ret_t rtn = rcutils_logging_set_logger_level("logger_name", RCUTILS_LOG_SEVERITY_DEBUG);
  }

  smapper_->configure(shared_from_this());
  this->declare_parameter<bool>("paused_new_measurements");
  // some systems dont have the tf transforms and lidar ready at startup which
  // produces a lot of spam. this allows the user to decide when they want to start
  // the lidar/TF processing.
  if (!this->get_parameter_or<bool>("paused_new_measurements", paused_at_startup_, false))
  {
    this->set_parameter({"paused_new_measurements", paused_at_startup_});
  }
}

/*****************************************************************************/
void SlamToolbox::setROSInterfaces()
/*****************************************************************************/
{
  double tmp_val = 30.;
  tmp_val        = this->declare_parameter("tf_buffer_duration", tmp_val);
  tf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(tmp_val));
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                                   get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

  pose_pub_   = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::Bool>("localization/status", 10);
  sst_        = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>(
    map_name_ + "_metadata", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ssMap_ = this->create_service<maidbot_std_srvs::srv::GetCompressedMap>(
    "slam_toolbox/dynamic_map", std::bind(&SlamToolbox::mapCallback, this, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3));
  ssPauseMeasurements_ = this->create_service<std_srvs::srv::Trigger>(
    "slam_toolbox/pause_new_measurements",
    std::bind(&SlamToolbox::pauseNewMeasurementsCallback, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3));
  ssSerialize_ = this->create_service<slam_toolbox::srv::SerializePoseGraph>(
    "slam_toolbox/serialize_map",
    std::bind(&SlamToolbox::serializePoseGraphCallback, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3));
  ssDesserialize_ = this->create_service<slam_toolbox::srv::DeserializePoseGraph>(
    "slam_toolbox/deserialize_map",
    std::bind(&SlamToolbox::deserializePoseGraphCallback, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3));
  ssStart_ = this->create_service<std_srvs::srv::Trigger>(
    "slam_toolbox/start",
    std::bind(&SlamToolbox::startSlamCallback, this, std::placeholders::_1, std::placeholders::_2));
  ssStop_ = this->create_service<std_srvs::srv::Trigger>(
    "slam_toolbox/stop",
    std::bind(&SlamToolbox::stopSlamCallback, this, std::placeholders::_1, std::placeholders::_2));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&SlamToolbox::laserCallback, this, std::placeholders::_1));
}

/*****************************************************************************/
void SlamToolbox::publishTransformLoop(const double& transform_publish_period)
/*****************************************************************************/
{
  if (transform_publish_period == 0)
  {
    return;
  }

  rclcpp::Rate r(1.0 / transform_publish_period);
  while (rclcpp::ok())
  {
    {
      boost::mutex::scoped_lock lock(map_to_odom_mutex_);
      rclcpp::Time scan_timestamp = scan_header.stamp;
      // Avoid publishing tf with initial 0.0 scan timestamp
      if (scan_timestamp.seconds() > 0.0 && !scan_header.frame_id.empty())
      {
        geometry_msgs::msg::TransformStamped msg;
        msg.transform       = tf2::toMsg(map_to_odom_);
        msg.child_frame_id  = odom_frame_;
        msg.header.frame_id = map_frame_;
        msg.header.stamp    = scan_timestamp + transform_timeout_;
        tfB_->sendTransform(msg);
      }
    }
    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::publishVisualizations()
/*****************************************************************************/
{
  nav_msgs::msg::OccupancyGrid& og = map_.map;
  og.info.resolution               = resolution_;
  og.info.origin.position.x        = 0.0;
  og.info.origin.position.y        = 0.0;
  og.info.origin.position.z        = 0.0;
  og.info.origin.orientation.x     = 0.0;
  og.info.origin.orientation.y     = 0.0;
  og.info.origin.orientation.z     = 0.0;
  og.info.origin.orientation.w     = 1.0;
  og.header.frame_id               = map_frame_;

  double map_update_interval = 10;
  map_update_interval        = this->declare_parameter("map_update_interval", map_update_interval);
  rclcpp::Rate r(1.0 / map_update_interval);

  while (rclcpp::ok())
  {
    updateMap();
    if (!isPaused(VISUALIZING_GRAPH))
    {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      closure_assistant_->publishGraph();
    }
    r.sleep();
  }
}

/*****************************************************************************/
void SlamToolbox::loadPoseGraphByParams()
/*****************************************************************************/
{
  std::string filename;
  geometry_msgs::msg::Pose2D pose;
  bool dock = false;
  if (shouldStartWithPoseGraph(filename, pose, dock))
  {
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Response>();
    req->initial_pose = pose;
    req->filename     = filename;
    if (dock)
    {
      req->match_type = slam_toolbox::srv::DeserializePoseGraph::Request::START_AT_FIRST_NODE;
    }
    else
    {
      req->match_type = slam_toolbox::srv::DeserializePoseGraph::Request::START_AT_GIVEN_POSE;
    }

    deserializePoseGraphCallback(nullptr, req, resp);
  }
}

/*****************************************************************************/
bool SlamToolbox::shouldStartWithPoseGraph(std::string& filename, geometry_msgs::msg::Pose2D& pose,
                                           bool& start_at_dock)
/*****************************************************************************/
{
  // if given a map to load at run time, do it.
  this->declare_parameter("map_file_name", std::string(""));
  auto map_start_pose =
    this->declare_parameter("map_start_pose", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  auto map_start_at_dock =
    this->declare_parameter("map_start_at_dock", rclcpp::ParameterType::PARAMETER_BOOL);
  filename = this->get_parameter("map_file_name").as_string();
  if (!filename.empty())
  {
    std::vector<double> read_pose;
    if (map_start_pose.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      read_pose     = map_start_pose.get<std::vector<double>>();
      start_at_dock = false;
      if (read_pose.size() != 3)
      {
        RCLCPP_ERROR(get_logger(), "LocalizationSlamToolbox: Incorrect "
                                   "number of arguments for map starting pose. Must be in format: "
                                   "[x, y, theta]. Starting at the origin");
        pose.x     = 0.;
        pose.y     = 0.;
        pose.theta = 0.;
      }
      else
      {
        pose.x     = read_pose[0];
        pose.y     = read_pose[1];
        pose.theta = read_pose[2];
      }
    }
    else if (map_start_at_dock.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      start_at_dock = map_start_at_dock.get<bool>();
    }
    else
    {
      RCLCPP_ERROR(get_logger(),
                   "LocalizationSlamToolbox: Map starting "
                   "pose not specified. Set either map_start_pose or map_start_at_dock.");
      return false;
    }

    return true;
  }

  return false;
}

/*****************************************************************************/
LaserRangeFinder* SlamToolbox::getLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
/*****************************************************************************/
{
  const std::string& frame = scan->header.frame_id;
  if (lasers_.find(frame) == lasers_.end())
  {
    try
    {
      lasers_[frame] = laser_assistant_->toLaserMetadata(*scan);
      dataset_->Add(lasers_[frame].getLaser(), true);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to compute laser pose, "
                   "aborting initialization (%s)",
                   e.what());
      return nullptr;
    }
  }

  return lasers_[frame].getLaser();
}

/*****************************************************************************/
bool SlamToolbox::waitForTransform(const std::string& scan_frame, const rclcpp::Time& stamp)
/*****************************************************************************/
{
  if (!tf_->_frameExists(odom_frame_))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf frame [%s] doesn't exist yet.'",
                         odom_frame_.c_str());
    return false;
  }

  if (!tf_->_frameExists(scan_frame))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf frame [%s] doesn't exist yet.'",
                         scan_frame.c_str());
    return false;
  }

  if (!tf_->canTransform(odom_frame_, scan_frame, stamp, transform_timeout_))
  {
    RCLCPP_WARN(get_logger(), "Failed to get transform %s -> %s.", scan_frame.c_str(),
                odom_frame_.c_str());
    return false;
  }

  return true;
}

/*****************************************************************************/
bool SlamToolbox::updateMap()
/*****************************************************************************/
{
  if (sst_->get_subscription_count() == 0)
  {
    return false;
  }
  boost::mutex::scoped_lock lock(smapper_mutex_);
  OccupancyGrid* occ_grid = smapper_->getOccupancyGrid(resolution_);
  if (!occ_grid)
  {
    return false;
  }

  // publish map as current
  map_.map.header.stamp = scan_header.stamp;
  sst_->publish(std::move(std::make_unique<nav_msgs::msg::OccupancyGrid>(map_.map)));
  sstm_->publish(std::move(std::make_unique<nav_msgs::msg::MapMetaData>(map_.map.info)));

  delete occ_grid;
  occ_grid = nullptr;
  return true;
}

/*****************************************************************************/
tf2::Stamped<tf2::Transform> SlamToolbox::setTransformFromPoses(
  const Pose2& corrected_pose, const Pose2& odom_pose, const rclcpp::Time& t,
  const bool& update_reprocessing_transform)
/*****************************************************************************/
{
  // Compute the map->odom transform
  tf2::Stamped<tf2::Transform> odom_to_map;
  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., corrected_pose.GetHeading());
  tf2::Stamped<tf2::Transform> base_to_map(
    tf2::Transform(q, tf2::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(),
    tf2_ros::fromMsg(t), base_frame_);
  try
  {
    geometry_msgs::msg::TransformStamped base_to_map_msg, odom_to_map_msg;

    // https://github.com/ros2/geometry2/issues/176
    // not working for some reason...
    // base_to_map_msg = tf2::toMsg(base_to_map);
    base_to_map_msg.header.stamp            = tf2_ros::toMsg(base_to_map.stamp_);
    base_to_map_msg.header.frame_id         = base_to_map.frame_id_;
    base_to_map_msg.transform.translation.x = base_to_map.getOrigin().getX();
    base_to_map_msg.transform.translation.y = base_to_map.getOrigin().getY();
    base_to_map_msg.transform.translation.z = base_to_map.getOrigin().getZ();
    base_to_map_msg.transform.rotation      = tf2::toMsg(base_to_map.getRotation());

    odom_to_map_msg = tf_->transform(base_to_map_msg, odom_frame_);
    tf2::fromMsg(odom_to_map_msg, odom_to_map);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_ERROR(get_logger(), "Transform from base_link to odom failed: %s", e.what());
    return odom_to_map;
  }

  // if we're continuing a previous session, we need to
  // estimate the homogenous transformation between the old and new
  // odometry frames and transform the new session
  // into the older session's frame
  if (update_reprocessing_transform)
  {
    tf2::Transform odom_to_base_serialized = base_to_map.inverse();
    tf2::Quaternion q1(0., 0., 0., 1.0);
    q1.setRPY(0., 0., tf2::getYaw(odom_to_base_serialized.getRotation()));
    odom_to_base_serialized.setRotation(q1);
    tf2::Transform odom_to_base_current = smapper_->toTfPose(odom_pose);
    reprocessing_transform_             = odom_to_base_serialized * odom_to_base_current.inverse();
  }

  // set map to odom for our transformation thread to publish
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  map_to_odom_ = tf2::Transform(tf2::Quaternion(odom_to_map.getRotation()),
                                tf2::Vector3(odom_to_map.getOrigin()))
                   .inverse();

  return odom_to_map;
}

/*****************************************************************************/
LocalizedRangeScan* SlamToolbox::getLocalizedRangeScan(
  LaserRangeFinder* laser, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
  Pose2& odom_pose)
/*****************************************************************************/
{
  // Create a vector of doubles for lib
  std::vector<kt_double> readings =
    laser_utils::scanToReadings(*scan, lasers_[scan->header.frame_id].isInverted());

  // transform by the reprocessing transform
  tf2::Transform pose_original       = smapper_->toTfPose(odom_pose);
  tf2::Transform tf_pose_transformed = reprocessing_transform_ * pose_original;
  Pose2 transformed_pose             = smapper_->toKartoPose(tf_pose_transformed);

  // create localized range scan
  LocalizedRangeScan* range_scan = new LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(transformed_pose);
  range_scan->SetCorrectedPose(transformed_pose);
  range_scan->SetTime(rclcpp::Time(scan->header.stamp).nanoseconds() / 1.e9);
  return range_scan;
}

/*****************************************************************************/
bool SlamToolbox::shouldProcessScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
                                    const Pose2& pose)
/*****************************************************************************/
{
  static Pose2 last_pose;
  static rclcpp::Time last_scan_time = rclcpp::Time(0.);
  static double min_dist2            = smapper_->getMapper()->getParamMinimumTravelDistance() *
                            smapper_->getMapper()->getParamMinimumTravelDistance();
  static int scan_ctr = 0;
  scan_ctr++;

  // we give it a pass on the first measurement to get the ball rolling
  if (first_measurement_)
  {
    last_scan_time     = scan->header.stamp;
    last_pose          = pose;
    first_measurement_ = false;
    return true;
  }

  // we are in a paused mode, reject incomming information
  if (isPaused(NEW_MEASUREMENTS))
  {
    return false;
  }

  // throttled out
  if ((scan_ctr % throttle_scans_) != 0)
  {
    return false;
  }

  // not enough time
  if (rclcpp::Time(scan->header.stamp) - last_scan_time < minimum_time_interval_)
  {
    return false;
  }

  // check moved enough, within 10% for correction error
  const double dist2 = last_pose.SquaredDistance(pose);
  if (dist2 < 0.8 * min_dist2 || scan_ctr < 5)
  {
    return false;
  }

  last_pose      = pose;
  last_scan_time = scan->header.stamp;

  return true;
}

/*****************************************************************************/
LocalizedRangeScan* SlamToolbox::addScan(LaserRangeFinder* laser, PosedScan& scan_w_pose)
/*****************************************************************************/
{
  return addScan(laser, scan_w_pose.scan, scan_w_pose.pose);
}

/*****************************************************************************/
LocalizedRangeScan* SlamToolbox::addScan(LaserRangeFinder* laser,
                                         const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
                                         Pose2& odom_pose)
/*****************************************************************************/
{
  static rclcpp::Time last_match_time = rclcpp::Time(0.);

  // get our localized range scan
  LocalizedRangeScan* range_scan = getLocalizedRangeScan(laser, scan, odom_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;

  Matrix3 covariance;
  covariance.SetToIdentity();

  // whether or not the scan was processed as only a scan match without updating
  // the graph and scan buffer
  bool match_only = false;

  if (processor_type_ == PROCESS)
  {
    boost::mutex::scoped_lock l(pose_mutex_);
    if (process_near_pose_)
    {
      range_scan->SetOdometricPose(*process_near_pose_);
      range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
      process_near_pose_.reset(nullptr);
    }
    processed = smapper_->getMapper()->Process(range_scan, &covariance);

    // if the scan was not processed into the map because of insuffcient travel
    // distance, then check if enough time as passed to just perform a scan
    // match without updating the graph or scan buffer
    rclcpp::Time stamp = scan->header.stamp;
    bool match_only    = !processed && maximum_match_interval_.seconds() >= 0.0 &&
                      stamp - last_match_time > maximum_match_interval_;
    if (match_only)
    {
      processed = smapper_->getMapper()->Process(range_scan, &covariance, true);
    }
  }
  else if (processor_type_ == PROCESS_FIRST_NODE)
  {
    processed                     = smapper_->getMapper()->ProcessAtDock(range_scan, &covariance);
    processor_type_               = PROCESS;
    update_reprocessing_transform = true;
  }
  else if (processor_type_ == PROCESS_NEAR_REGION ||
           (processor_type_ == PROCESS_LOCALIZATION && process_near_pose_))
  {
    boost::mutex::scoped_lock l(pose_mutex_);
    if (!process_near_pose_)
    {
      RCLCPP_ERROR(get_logger(), "Process near region called without a "
                                 "valid region request. Ignoring scan.");
      return nullptr;
    }
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan, false, &covariance);
    update_reprocessing_transform = true;

    if (processor_type_ != PROCESS_LOCALIZATION)
    {
      processor_type_ = PROCESS;
    }
  }
  else if (processor_type_ == PROCESS_LOCALIZATION)
  {
    processed = smapper_->getMapper()->ProcessLocalization(range_scan, &covariance, match_only);

    rclcpp::Time stamp = scan->header.stamp;
    bool match_only    = !processed && maximum_match_interval_.seconds() >= 0.0 &&
                      stamp - last_match_time > maximum_match_interval_;
    if (match_only)
    {
      processed = smapper_->getMapper()->ProcessLocalization(range_scan, &covariance, true);
    }
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "SlamToolbox: No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  // and add our scan to storage
  if (processed)
  {
    last_match_time = scan->header.stamp;

    setTransformFromPoses(range_scan->GetCorrectedPose(), odom_pose, scan->header.stamp,
                          update_reprocessing_transform);
    if (processor_type_ != PROCESS_LOCALIZATION)
    {
      // localization bookkeeping clashes with dataset bookkeeping, so best to
      // avoid using them together.
      dataset_->Add(range_scan);
    }

    publishPose(range_scan->GetCorrectedPose(), covariance, scan->header.stamp);
  }
  else
  {
    delete range_scan;
    range_scan = nullptr;
  }

  return range_scan;
}

/*****************************************************************************/
void SlamToolbox::publishPose(const Pose2& pose, const Matrix3& cov, const rclcpp::Time& t)
/*****************************************************************************/
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp    = t;
  pose_msg.header.frame_id = map_frame_;

  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., pose.GetHeading());
  tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
  tf2::toMsg(transform, pose_msg.pose.pose);

  pose_msg.pose.covariance[0]  = cov(0, 0) * position_covariance_scale_; // x
  pose_msg.pose.covariance[1]  = cov(0, 1) * position_covariance_scale_; // xy
  pose_msg.pose.covariance[6]  = cov(1, 0) * position_covariance_scale_; // xy
  pose_msg.pose.covariance[7]  = cov(1, 1) * position_covariance_scale_; // y
  pose_msg.pose.covariance[35] = cov(2, 2) * yaw_covariance_scale_;      // yaw

  pose_pub_->publish(pose_msg);

  std_msgs::msg::Bool status;
  status.data = true;
  status_pub_->publish(status);
}

/*****************************************************************************/
bool SlamToolbox::mapCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<maidbot_std_srvs::srv::GetCompressedMap::Request> req,
  std::shared_ptr<maidbot_std_srvs::srv::GetCompressedMap::Response> res)
/*****************************************************************************/
{
  if (map_.map.info.width && map_.map.info.height)
  {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    maidbot_msg_utils::compressOccupancyGrid(map_.map, res->map);
    res->success = true;
    return true;
  }
  else
  {
    res->success = false;
    return false;
  }
}

/*****************************************************************************/
bool SlamToolbox::pauseNewMeasurementsCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
/*****************************************************************************/
{
  toggleScanProcessing();
  resp->success = true;
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
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (!slam_running_)
  {
    RCLCPP_WARN(get_logger(), "serializePoseGraphCallback: Ignoring request to serialize pose "
                              "graph because SLAM is not running.");
    return false;
  }
  std::string filename = req->filename;

  // if we're inside the snap, we need to write to commonly accessible space
  if (snap_utils::isInSnap())
  {
    filename = snap_utils::getSnapPath() + std::string("/") + filename;
  }

  boost::mutex::scoped_lock lock(smapper_mutex_);
  serialization::write(filename, *smapper_->getMapper(), *dataset_, shared_from_this());
  return true;
}

/*****************************************************************************/
void SlamToolbox::loadSerializedPoseGraph(std::unique_ptr<Mapper>& mapper,
                                          std::unique_ptr<Dataset>& dataset)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(smapper_mutex_);

  solver_->Reset();

  // add the nodes and constraints to the optimizer
  VerticeMap mapper_vertices         = mapper->GetGraph()->GetVertices();
  VerticeMap::iterator vertex_map_it = mapper_vertices.begin();
  for (vertex_map_it; vertex_map_it != mapper_vertices.end(); ++vertex_map_it)
  {
    ScanMap::iterator vertex_it = vertex_map_it->second.begin();
    for (vertex_it; vertex_it != vertex_map_it->second.end(); ++vertex_it)
    {
      if (vertex_it->second != nullptr)
      {
        solver_->AddNode(vertex_it->second);
      }
    }
  }

  EdgeVector mapper_edges       = mapper->GetGraph()->GetEdges();
  EdgeVector::iterator edges_it = mapper_edges.begin();
  for (edges_it; edges_it != mapper_edges.end(); ++edges_it)
  {
    if (*edges_it != nullptr)
    {
      solver_->AddConstraint(*edges_it);
    }
  }

  mapper->SetScanSolver(solver_.get());

  // move the memory to our working dataset
  smapper_->setMapper(mapper.release());
  smapper_->configure(shared_from_this());
  closure_assistant_->setMapper(smapper_->getMapper());
  dataset_.reset(dataset.release());

  if (!smapper_->getMapper())
  {
    RCLCPP_FATAL(get_logger(), "loadSerializedPoseGraph: Could not properly load "
                               "a valid mapping object. Did you modify something by hand?");
    exit(-1);
  }

  if (dataset_->GetLasers().size() < 1)
  {
    RCLCPP_FATAL(get_logger(), "loadSerializedPoseGraph: Cannot deserialize "
                               "dataset with no laser objects.");
    exit(-1);
  }

  // create a current laser sensor
  LaserRangeFinder* laser = dynamic_cast<LaserRangeFinder*>(dataset_->GetLasers()[0]);
  Sensor* pSensor         = dynamic_cast<Sensor*>(laser);
  if (pSensor)
  {
    SensorManager::GetInstance()->RegisterSensor(pSensor);
    lasers_.clear();
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Invalid sensor pointer in dataset."
                               " Unable to register sensor.");
  }

  solver_->Compute();
}

/*****************************************************************************/
bool SlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == slam_toolbox::srv::DeserializePoseGraph::Request::UNSET)
  {
    RCLCPP_ERROR(get_logger(), "Deserialization called without valid"
                               " processor type set. Undefined behavior!");
    return false;
  }

  if (slam_running_)
  {
    RCLCPP_WARN(get_logger(), "deserializePoseGraphCallback: Ignoring request to deserialize pose "
                              "graph because SLAM is running.");
    return false;
  }

  std::string filename = req->filename;

  if (filename.empty())
  {
    RCLCPP_WARN(get_logger(), "No map file given!");
    return true;
  }

  // if we're inside the snap, we need to write to commonly accessible space
  if (snap_utils::isInSnap())
  {
    filename = snap_utils::getSnapPath() + std::string("/") + filename;
  }

  std::unique_ptr<Dataset> dataset = std::make_unique<Dataset>();
  std::unique_ptr<Mapper> mapper   = std::make_unique<Mapper>();

  if (!serialization::read(filename, *mapper, *dataset, shared_from_this()))
  {
    RCLCPP_ERROR(get_logger(),
                 "DeserializePoseGraph: Failed to read "
                 "file: %s.",
                 filename.c_str());
    return true;
  }
  RCLCPP_INFO(get_logger(), "DeserializePoseGraph: Successfully read file.");

  loadSerializedPoseGraph(mapper, dataset);
  updateMap();

  first_measurement_ = true;
  boost::mutex::scoped_lock l(pose_mutex_);
  switch (req->match_type)
  {
    case procType::START_AT_FIRST_NODE:
      processor_type_ = PROCESS_FIRST_NODE;
      break;
    case procType::START_AT_GIVEN_POSE:
      processor_type_ = PROCESS_NEAR_REGION;
      process_near_pose_ =
        std::make_unique<Pose2>(req->initial_pose.x, req->initial_pose.y, req->initial_pose.theta);
      break;
    case procType::LOCALIZE_AT_POSE:
      processor_type_ = PROCESS_LOCALIZATION;
      process_near_pose_ =
        std::make_unique<Pose2>(req->initial_pose.x, req->initial_pose.y, req->initial_pose.theta);
      break;
    default:
      RCLCPP_FATAL(get_logger(), "Deserialization called without valid processor type set.");
  }

  return true;
}

/*****************************************************************************/
void SlamToolbox::toggleScanProcessing()
/*****************************************************************************/
{
  bool curr_state = isPaused(NEW_MEASUREMENTS);
  state_.set(NEW_MEASUREMENTS, !curr_state);

  if (isPaused(NEW_MEASUREMENTS))
  {
    scan_sub_.reset();
  }
  else
  {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&SlamToolbox::laserCallback, this, std::placeholders::_1));
  }

  this->set_parameter({"paused_new_measurements", !curr_state});
  RCLCPP_INFO(get_logger(), "SlamToolbox: Toggled to %s",
              !curr_state ? "pause taking new measurements." : "actively taking new measurements.");
}
/*****************************************************************************/
void SlamToolbox::startSlamCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
/*****************************************************************************/
{
  if (slam_running_)
  {
    RCLCPP_WARN(get_logger(), "SLAM was already started!");
    resp->success = false;
    return;
  }
  if (isPaused(NEW_MEASUREMENTS))
  {
    toggleScanProcessing();
  }
  slam_running_ = true;
  resp->success = true;
}
/*****************************************************************************/
void SlamToolbox::stopSlamCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
/*****************************************************************************/
{
  if (!slam_running_)
  {
    RCLCPP_WARN(get_logger(), "SLAM was already stopped!");
    resp->success = false;
    return;
  }
  if (!isPaused(NEW_MEASUREMENTS))
  {
    toggleScanProcessing();
  }
  resetSlam();
  slam_running_ = false;
  resp->success = true;
}
/*****************************************************************************/
void SlamToolbox::resetSlam()
/*****************************************************************************/
{
  const bool paused_before_reset = isPaused(NEW_MEASUREMENTS);

  // pause new scan processing
  if (!isPaused(NEW_MEASUREMENTS))
  {
    toggleScanProcessing();
  }

  RCLCPP_WARN(get_logger(), "Starting SLAM reset.");
  boost::mutex::scoped_lock lock(smapper_mutex_);
  if (smapper_ && processor_type_ == ProcessType::PROCESS_LOCALIZATION)
  {
    smapper_->clearLocalizationBuffer();
  }

  dataset_->Clear();

  std::unique_ptr<Mapper> mapper = std::make_unique<Mapper>();

  solver_->Reset();
  mapper->SetScanSolver(solver_.get());

  // move the memory to our working dataset
  smapper_->setMapper(mapper.release());
  smapper_->configure(shared_from_this());
  // reset the closure_assistant's mapper (since its address just changed)
  closure_assistant_->setMapper(smapper_->getMapper());

  // reset slam_toolbox bookkeeping
  lasers_.clear();
  scan_holder_->clearScans();
  first_measurement_ = true;
  // no need to lock since we paused scan callbacks
  processor_type_ = PROCESS;
  process_near_pose_.reset();

  nav_msgs::msg::OccupancyGrid& og = map_.map;
  og.info.resolution               = resolution_;
  og.info.origin.position.x        = 0.0;
  og.info.origin.position.y        = 0.0;
  og.info.origin.position.z        = 0.0;
  og.info.origin.orientation.x     = 0.0;
  og.info.origin.orientation.y     = 0.0;
  og.info.origin.orientation.z     = 0.0;
  og.info.origin.orientation.w     = 1.0;
  og.header.frame_id               = map_frame_;

  // resume new scan processing (unless it was already paused)
  if (isPaused(NEW_MEASUREMENTS) && !paused_before_reset)
  {
    toggleScanProcessing();
  }
  RCLCPP_WARN(get_logger(), "Finished SLAM reset.");
}

} // namespace slam_toolbox
