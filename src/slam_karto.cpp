/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright Work Modifications (c) 2017, Simbe Robotics, Inc.
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
/* Modified: Steven Macenski */

#include <slam_karto/slam_karto.hpp>

/*****************************************************************************/
SlamKarto::SlamKarto() : laser_count_(0),
                         transform_thread_(NULL),
                         run_thread_(NULL),
                         visualization_thread_(NULL),
                         solver_loader_("slam_karto", "karto::ScanSolver"),
                         last_scan_time_(0),
                         paused_(false),
                         tf_(ros::Duration(14400.)) // 4 hours
/*****************************************************************************/
{
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  ros::NodeHandle private_nh_("~");
  setParams(private_nh_);
  setSolver(private_nh_);
  setROSInterfaces();

  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);
  transform_thread_ = new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period));
  run_thread_ = new boost::thread(boost::bind(&SlamKarto::Run, this));
  visualization_thread_ = new boost::thread(boost::bind(&SlamKarto::publishVisualizations, this));
}

/*****************************************************************************/
void SlamKarto::setSolver(ros::NodeHandle& private_nh_)
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
void SlamKarto::setParams(ros::NodeHandle& private_nh_)
/*****************************************************************************/
{
  map_to_odom_.setIdentity();

  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
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
    map_update_interval = 5.0;
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
  // Setting General Parameters from the Parameter Server
  bool use_scan_matching;
  if(private_nh_.getParam("use_scan_matching", use_scan_matching))
    mapper_->setParamUseScanMatching(use_scan_matching);
  
  bool use_scan_barycenter;
  if(private_nh_.getParam("use_scan_barycenter", use_scan_barycenter))
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_travel_distance;
  if(private_nh_.getParam("minimum_travel_distance", minimum_travel_distance))
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

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

  // Setting Correlation Parameters from the Parameter Server
  double correlation_search_space_dimension;
  if(private_nh_.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution;
  if(private_nh_.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation;
  if(private_nh_.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server
  double loop_search_space_dimension;
  if(private_nh_.getParam("loop_search_space_dimension", loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if(private_nh_.getParam("loop_search_space_resolution", loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation;
  if(private_nh_.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server
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
void SlamKarto::setROSInterfaces()
/*****************************************************************************/
{
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ssMap_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  ssPause_ = node_.advertiseService("pause", &SlamKarto::pauseCallback, this);
  ssClear_ = node_.advertiseService("clear_queue", &SlamKarto::clearQueueCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("graph_visualization",1);
}

/*****************************************************************************/
SlamKarto::~SlamKarto()
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
  std::map<std::string, karto::LaserRangeFinder*>::iterator it = lasers_.begin();
  for (it; it!=lasers_.end(); ++it)
  {
    delete it->second;
  }
}

/*****************************************************************************/
void SlamKarto::publishLoop(double transform_publish_period)
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
void SlamKarto::publishVisualizations()
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
  try
  {
    while(ros::ok())
    {
      ROS_INFO("looping");
      updateMap();
      publishGraphVisualization();
      r.sleep();
    }
  }
  catch (...)
  {
    ROS_ERROR("visualization thread died, segfault");
  }
  ROS_ERROR("visualization thread died ");
}

/*****************************************************************************/
karto::LaserRangeFinder* SlamKarto::getLaser(const \
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

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
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
      ROS_INFO("laser is mounted upside-down");

    // Create a laser range finder device and copy in data from the first scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, \
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
    lasers_[scan->header.frame_id] = laser;
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

/*****************************************************************************/
bool SlamKarto::getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
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
          karto::Pose2(odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw);
  return true;
}

/*****************************************************************************/
void SlamKarto::publishGraphVisualization()
/*****************************************************************************/
{
  if (marker_publisher_.getNumSubscribers() == 0)
  {
    return;
  }

  std::vector<Eigen::Vector2d> graph;
  solver_->getGraph(graph);

  visualization_msgs::MarkerArray marray;

  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "2d_slam_toolbox";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.action = visualization_msgs::Marker::ADD;
  m.lifetime = ros::Duration(0.);

  uint i=0;
  for (i; i<graph.size(); i++) 
  {
    m.id = i;
    m.pose.position.x = graph[i](0);
    m.pose.position.y = graph[i](1);
    marray.markers.push_back(visualization_msgs::Marker(m));
  }
  marker_publisher_.publish(marray);
}

/*****************************************************************************/
void SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  if(isPaused())
  {
    // we are in a paused mode, reject incomming information
    return;
  }

  // not enough time
  if ( (scan->header.stamp.toSec() - last_scan_time_ ) < minimum_time_interval_)
  {
    return;
  }

  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  // no odom info
  karto::Pose2 pose;
  if(!getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // ok... maybe valid
  q_.push(posed_scan(scan, pose));
  last_scan_time_ = scan->header.stamp.toSec(); 
  return;
}

/*****************************************************************************/
bool SlamKarto::updateMap()
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
    occ_grid = karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), 
                                                     resolution_);
  }
  
  if(!occ_grid)
  {
    return false;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

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
bool SlamKarto::addScan(karto::LaserRangeFinder* laser,
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
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> (tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
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
bool SlamKarto::mapCallback(nav_msgs::GetMap::Request  &req,
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
bool SlamKarto::pauseCallback(slam_karto::Pause::Request  &req,
                              slam_karto::Pause::Response &resp)
/*****************************************************************************/
{
  togglePause();
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SlamKarto::clearQueueCallback(slam_karto::ClearQueue::Request& req,
                                   slam_karto::ClearQueue::Response& resp)
/*****************************************************************************/
{
  std::queue<posed_scan> empty;
  std::swap( q_, empty );
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SlamKarto::isPaused()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(pause_mutex_);
  return paused_;
}

/*****************************************************************************/
void SlamKarto::togglePause()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(pause_mutex_);
  paused_ = !paused_;
  ROS_INFO("SlamKarto Toggled to %s", isPaused() ? "paused." : "active.");
}

/*****************************************************************************/
void SlamKarto::Run()
/*****************************************************************************/
{
  ros::Rate r(60);

  while(ros::ok())
  {
    if (!q_.empty())
    {
      posed_scan scan_w_pose = q_.front();
      q_.pop();
      ROS_INFO_THROTTLE(15., "Queue size: %i", (int)q_.size());

      // Check whether we know about this laser yet
      karto::LaserRangeFinder* laser = getLaser(scan_w_pose.scan);

      if(!laser)
      {
        ROS_WARN("Failed to create laser device for %s; discarding scan",
           scan_w_pose.scan->header.frame_id.c_str());
        break;
      }
      addScan(laser, scan_w_pose.scan, scan_w_pose.pose);
    }
    r.sleep();
  }
}

/*****************************************************************************/
int main(int argc, char** argv)
/*****************************************************************************/
{
  ros::init(argc, argv, "slam_karto");
  SlamKarto kn;
  ros::spin();
  return 0;
}
