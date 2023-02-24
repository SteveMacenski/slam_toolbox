/*
 * slam_toolbox
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
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

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_COMMON_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_COMMON_H_

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "pluginlib/class_loader.h"

#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/slam_mapper.hpp"
#include "slam_toolbox/snap_utils.hpp"
#include "slam_toolbox/laser_utils.hpp"
#include "slam_toolbox/get_pose_helper.hpp"
#include "slam_toolbox/map_saver.hpp"
#include "slam_toolbox/loop_closure_assistant.hpp"

#include <string>
#include <map>
#include <vector>
#include <queue>
#include <cstdlib>
#include <fstream>
#include <boost/thread.hpp>
#include <sys/resource.h>

namespace slam_toolbox
{

// dirty, dirty cheat I love
using namespace ::toolbox_types;

class SlamToolbox
{
public:
  SlamToolbox(ros::NodeHandle& nh);
  ~SlamToolbox();

protected:
  // threads
  void publishVisualizations();
  void publishTransformLoop(const double& transform_publish_period);

  // setup
  void setParams(ros::NodeHandle& nh);
  void setSolver(ros::NodeHandle& private_nh_);
  void setROSInterfaces(ros::NodeHandle& node);

  // callbacks
  virtual void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) = 0;
  bool mapCallback(nav_msgs::GetMap::Request& req,
    nav_msgs::GetMap::Response& res);
  virtual bool serializePoseGraphCallback(slam_toolbox_msgs::SerializePoseGraph::Request& req,
    slam_toolbox_msgs::SerializePoseGraph::Response& resp);
  virtual bool deserializePoseGraphCallback(slam_toolbox_msgs::DeserializePoseGraph::Request& req,
    slam_toolbox_msgs::DeserializePoseGraph::Response& resp);
  void loadSerializedPoseGraph(std::unique_ptr<karto::Mapper>&, std::unique_ptr<karto::Dataset>&);
  void loadPoseGraphByParams(ros::NodeHandle& nh);

  // functional bits
  karto::LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
  virtual karto::LocalizedRangeScan* addScan(karto::LaserRangeFinder* laser, const sensor_msgs::LaserScan::ConstPtr& scan,
    karto::Pose2& karto_pose);
  karto::LocalizedRangeScan* addScan(karto::LaserRangeFinder* laser, PosedScan& scanWPose);
  bool updateMap();
  tf2::Stamped<tf2::Transform> setTransformFromPoses(const karto::Pose2& pose,
    const karto::Pose2& karto_pose, const ros::Time& t, const bool& update_reprocessing_transform);
  karto::LocalizedRangeScan* getLocalizedRangeScan(karto::LaserRangeFinder* laser,
    const sensor_msgs::LaserScan::ConstPtr& scan,
    karto::Pose2& karto_pose);
  bool shouldStartWithPoseGraph(std::string& filename, geometry_msgs::Pose2D& pose, bool& start_at_dock);
  bool shouldProcessScan(const sensor_msgs::LaserScan::ConstPtr& scan, const karto::Pose2& pose);
  void publishPose(const karto::Pose2 & pose, const karto::Matrix3 & cov, const ros::Time & t);

  // pausing bits
  bool isPaused(const PausedApplication& app);
  bool pauseNewMeasurementsCallback(slam_toolbox_msgs::Pause::Request& req,
    slam_toolbox_msgs::Pause::Response& resp);

  // ROS-y-ness
  ros::NodeHandle nh_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfL_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > scan_filter_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan> > scan_filter_;
  ros::Publisher sst_, sstm_, pose_pub_;
  ros::ServiceServer ssMap_, ssPauseMeasurements_, ssSerialize_, ssDesserialize_;

  // Storage for ROS parameters
  std::string odom_frame_, map_frame_, base_frame_, map_name_, scan_topic_;
  ros::Duration transform_timeout_, tf_buffer_dur_, minimum_time_interval_;
  int throttle_scans_;

  double resolution_;
  double position_covariance_scale_;
  double yaw_covariance_scale_;
  bool first_measurement_, enable_interactive_mode_;

  // Book keeping
  std::unique_ptr<mapper_utils::SMapper> smapper_;
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
  boost::mutex map_to_odom_mutex_, smapper_mutex_, pose_mutex_;
  PausedState state_;
  nav_msgs::GetMap::Response map_;
  ProcessType processor_type_;
  std::unique_ptr<karto::Pose2> process_near_pose_;
  tf2::Transform reprocessing_transform_;

  // pluginlib
  pluginlib::ClassLoader<karto::ScanSolver> solver_loader_;
  boost::shared_ptr<karto::ScanSolver> solver_;
};

} // end namespace

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_COMMON_H_
