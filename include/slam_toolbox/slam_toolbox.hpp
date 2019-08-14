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

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

#include "interactive_markers/interactive_marker_server.h"
#include "interactive_markers/menu_handler.h"
#include "pluginlib/class_loader.h"

#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/mapper_utils.hpp"
#include "slam_toolbox/snap_utils.hpp"

#include <string>
#include <map>
#include <vector>
#include <queue>
#include <cstdlib>
#include <fstream>
#include <boost/thread.hpp>

namespace slam_toolbox
{

// dirty, dirty cheat I love
using namespace ::toolbox_types;

class SlamToolbox
{
public:
  SlamToolbox();
  ~SlamToolbox();

private:
  // threads
  void Run();
  void PublishVisualizations();
  void PublishTransformLoop(const double& transform_publish_period);

  // setup
  void SetParams(ros::NodeHandle& nh);
  void SetSolver(ros::NodeHandle& private_nh_);
  void SetROSInterfaces(ros::NodeHandle& node);

  // callbacks
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool MapCallback(nav_msgs::GetMap::Request  &req,
                   nav_msgs::GetMap::Response &res);
  bool PauseProcessingCallback(slam_toolbox::Pause::Request& req,
                     slam_toolbox::Pause::Response& resp);
  bool PauseNewMeasurementsCallback(slam_toolbox::Pause::Request& req,
                     slam_toolbox::Pause::Response& resp);
  bool ClearQueueCallback(slam_toolbox::ClearQueue::Request& req,
                          slam_toolbox::ClearQueue::Response& resp);
  bool InteractiveCallback(slam_toolbox::ToggleInteractive::Request  &req,
                           slam_toolbox::ToggleInteractive::Response &resp);
  bool ClearChangesCallback(slam_toolbox::Clear::Request  &req,
                            slam_toolbox::Clear::Response &resp);
  bool SaveMapCallback(slam_toolbox::SaveMap::Request  &req,
                       slam_toolbox::SaveMap::Response &resp);
  bool ManualLoopClosureCallback(slam_toolbox::LoopClosure::Request  &req,
                                 slam_toolbox::LoopClosure::Response &resp);
  bool SerializePoseGraphCallback(slam_toolbox::SerializePoseGraph::Request  &req,
                                  slam_toolbox::SerializePoseGraph::Response &resp);
  bool DeserializePoseGraphCallback(slam_toolbox::DeserializePoseGraph::Request &req,
                                    slam_toolbox::DeserializePoseGraph::Response &resp);
  void LocalizePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  // functional bits
  bool GetOdomPose(karto::Pose2& karto_pose, const ros::Time& t);
  karto::LaserRangeFinder* GetLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool AddScan(karto::LaserRangeFinder* laser,
               const sensor_msgs::LaserScan::ConstPtr& scan,
               karto::Pose2& karto_pose);
  bool UpdateMap();
  void PublishGraph();
  void MoveNode(const int& id, const Eigen::Vector3d& pose, const bool correct = true);
  void ProcessInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void ClearMovedNodes();
  void AddMovedNodes(const int& id, Eigen::Vector3d vec);

  // state
  bool IsPaused(const PausedApplication& app);

  // ROS-y-ness
  ros::NodeHandle nh_;
  std::unique_ptr<tf::TransformListener> tf_;
  std::unique_ptr<tf::TransformBroadcaster> tfB_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > scan_filter_sub_;
  ros::Subscriber localization_pose_sub_;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan> > scan_filter_;
  ros::Publisher sst_, sstm_, marker_publisher_, scan_publisher_;
  ros::ServiceServer ssMap_, ssClear_, ssInteractive_, ssLoopClosure_, ssPause_processing_, ssPause_measurements_, ssClear_manual_, ssSave_map_, ssSerialize_, ssLoadMap_;
  nav_msgs::GetMap::Response map_;

  // Storage for ROS parameters
  std::string odom_frame_, map_frame_, base_frame_, laser_frame_;
  int throttle_scans_;
  ros::Duration transform_timeout_;
  double resolution_, minimum_time_interval_, minimum_travel_distance_;
  bool publish_occupancy_map_, first_measurement_, sychronous_, online_;
  ProcessType processor_type_;
  geometry_msgs::Pose2D process_near_region_pose_;
  tf::Transform reprocessing_transform_;

  // Book keeping
  std::unique_ptr<karto::Mapper> mapper_;
  std::unique_ptr<karto::Dataset> dataset_;
  std::map<std::string, laserMetadata> lasers_;

  // Internal state
  std::unique_ptr<boost::thread> transform_thread_, run_thread_, visualization_thread_;
  tf::Transform map_to_odom_;
  bool inverted_laser_, pause_graph_, pause_processing_, pause_new_measurements_, interactive_mode_, localization_pose_set_;
  std::queue<posedScan> q_;
  boost::mutex map_mutex_, pause_mutex_, map_to_odom_mutex_, mapper_mutex_, interactive_mutex_, moved_nodes_mutex_;

  // visualization
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  std::map<int, Eigen::Vector3d> moved_nodes_;
  std::vector<sensor_msgs::LaserScan> current_scans_;

  // pluginlib
  pluginlib::ClassLoader<karto::ScanSolver> solver_loader_;
  boost::shared_ptr<karto::ScanSolver> solver_;
};

} // end namespace
