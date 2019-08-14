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

#include <string>
#include <map>
#include <memory>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "interactive_markers/interactive_marker_server.h"
#include "interactive_markers/menu_handler.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "karto_sdk/Mapper.h"
#include "slam_toolbox/toolbox_types.hpp"

using namespace toolbox_types;

class MergeMapsKinematic
{

public:
  MergeMapsKinematic();
  ~MergeMapsKinematic();

private:

  // setup
  void Setup();

  // callback
  bool MergeMapCallback(slam_toolbox::MergeMaps::Request  &req, slam_toolbox::MergeMaps::Response &resp);
  bool AddSubmapCallback(slam_toolbox::AddSubmap::Request  &req, slam_toolbox::AddSubmap::Response &resp);
  void ProcessInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void KartoToROSOccupancyGrid(const karto::LocalizedRangeScanVector& scans, nav_msgs::GetMap::Response& map);
  typedef std::vector<karto::LocalizedRangeScanVector>::iterator localized_range_scans_vec_it;
  typedef karto::LocalizedRangeScanVector::iterator localized_range_scans_it;

  // ROS-y-ness
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> sstS_, sstmS_;
  ros::ServiceServer ssMap_, ssSubmap_;

  // param and state
  std::string map_frame_;
  double resolution_;
  double max_laser_range_;
  int num_submaps_;

  //karto bookkeeping
  std::map<std::string, laserMetadata> lasers_;
  std::vector<std::unique_ptr<karto::Dataset> > dataset_vec_;

  // TF
  std::unique_ptr<tf::TransformBroadcaster> tfB_;

  // visualization
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  std::map<int, Eigen::Vector3d> submap_locations_;
  std::vector<karto::LocalizedRangeScanVector> scans_vec_;
  std::map<int, tf::Transform> submap_marker_transform_;

  //apply transformation to correct pose
  karto::Pose2 ApplyCorrection(const karto::Pose2& pose, const tf::Transform& submap_correction);
  karto::Vector2<kt_double> ApplyCorrection(const karto::Vector2<kt_double>&  pose, const tf::Transform& submap_correction);
};
