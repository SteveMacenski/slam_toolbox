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

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_H_

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "pluginlib/class_loader.h"

#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/mapper_utils.hpp"
#include "slam_toolbox/snap_utils.hpp"
#include "slam_toolbox/laser_utils.hpp"
#include "slam_toolbox/pose_utils.hpp"
#include "slam_toolbox/map_saver.hpp"
#include "slam_toolbox/visualization_utils.hpp"
#include "slam_toolbox/loop_closure_assistant.hpp"

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
  void run();
  void publishVisualizations();
  void publishTransformLoop(const double& transform_publish_period);

  // setup
  void setParams(ros::NodeHandle& nh);
  void setSolver(ros::NodeHandle& private_nh_);
  void setROSInterfaces(ros::NodeHandle& node);

  // callbacks
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool mapCallback(nav_msgs::GetMap::Request  &req,
                   nav_msgs::GetMap::Response &res);
  bool clearQueueCallback(slam_toolbox::ClearQueue::Request& req,
                          slam_toolbox::ClearQueue::Response& resp);
  bool serializePoseGraphCallback(slam_toolbox::SerializePoseGraph::Request  &req,
                                  slam_toolbox::SerializePoseGraph::Response &resp);
  bool deserializePoseGraphCallback(slam_toolbox::DeserializePoseGraph::Request &req,
                                    slam_toolbox::DeserializePoseGraph::Response &resp);
  void loadSerializedPoseGraph(std::unique_ptr<karto::Mapper>&, std::unique_ptr<karto::Dataset>&);
  void localizePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  // functional bits
  karto::LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool addScan(karto::LaserRangeFinder* laser,
               const sensor_msgs::LaserScan::ConstPtr& scan,
               karto::Pose2& karto_pose);
  bool updateMap();
  void publishGraph();
  bool shouldProcessScan(const sensor_msgs::LaserScan::ConstPtr& scan, const karto::Pose2& pose);

  // TODO state helper
  bool isPaused(const PausedApplication& app);
  bool pauseProcessingCallback(slam_toolbox::Pause::Request& req,
                     slam_toolbox::Pause::Response& resp);
  bool pauseNewMeasurementsCallback(slam_toolbox::Pause::Request& req,
                     slam_toolbox::Pause::Response& resp);
  bool pauseCallback(slam_toolbox::ToggleInteractive::Request  &req,
                     slam_toolbox::ToggleInteractive::Response &resp);

  // ROS-y-ness
  ros::NodeHandle nh_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfL_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > scan_filter_sub_;
  ros::Subscriber localization_pose_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan> > scan_filter_;
  ros::Publisher sst_, sstm_, marker_publisher_;
  ros::ServiceServer ssMap_, ssClear_, ssInteractive_, ssPause_measurements_, ssSerialize_, ssLoadMap_;
  nav_msgs::GetMap::Response map_;

  // Storage for ROS parameters
  std::string odom_frame_, map_frame_, base_frame_, laser_frame_, map_name_;
  int throttle_scans_;
  ros::Duration transform_timeout_, tf_buffer_dur_, minimum_time_interval_;
  double resolution_, minimum_travel_distance_;
  bool publish_occupancy_map_, first_measurement_, sychronous_;
  ProcessType processor_type_;
  geometry_msgs::Pose2D process_near_pose_;
  tf2::Transform reprocessing_transform_;

  // Book keeping
  std::unique_ptr<karto::Mapper> mapper_;
  std::unique_ptr<karto::Dataset> dataset_;
  std::map<std::string, laser_utils::LaserMetadata> lasers_;

  // helpers
  std::unique_ptr<laser_utils::LaserAssistant> laser_assistant_;
  std::unique_ptr<pose_utils::GetPoseHelper> pose_helper_;
  std::unique_ptr<map_saver::MapSaver> map_saver_;
  std::unique_ptr<loop_closure_assistant::LoopClosureAssistant> closure_assistant_;
  std::unique_ptr<laser_utils::ScanHolder> scan_holder_;

  // Internal state
  std::vector<std::unique_ptr<boost::thread> > threads_;
  tf2::Transform map_to_odom_;
  bool pause_graph_, pause_processing_, pause_new_measurements_, interactive_mode_, localization_pose_set_;
  std::queue<posedScan> q_;
  boost::mutex map_mutex_, pause_mutex_, map_to_odom_mutex_, mapper_mutex_, interactive_mutex_;

  // visualization
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;

  // pluginlib
  pluginlib::ClassLoader<karto::ScanSolver> solver_loader_;
  boost::shared_ptr<karto::ScanSolver> solver_;
};

} // end namespace

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_H_
