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

#ifndef SLAM_TOOLBOX__MERGE_MAPS_KINEMATIC_HPP_
#define SLAM_TOOLBOX__MERGE_MAPS_KINEMATIC_HPP_

#include <thread>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <map>
#include <memory>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "../lib/karto_sdk/include/karto_sdk/Mapper.h"
#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/toolbox_msgs.hpp"
#include "slam_toolbox/laser_utils.hpp"
#include "slam_toolbox/visualization_utils.hpp"

using namespace toolbox_types;  // NOLINT
using namespace karto;  // NOLINT

class MergeMapsKinematic : public rclcpp::Node
{
  typedef std::vector<karto::LocalizedRangeScanVector>::iterator LocalizedRangeScansVecIt;
  typedef karto::LocalizedRangeScanVector::iterator LocalizedRangeScansIt;

public:
  MergeMapsKinematic();
  ~MergeMapsKinematic();
  // setup
  void configure();

private:
  // callback
  bool mergeMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::MergeMaps::Request> req,
    std::shared_ptr<slam_toolbox::srv::MergeMaps::Response> resp);
  bool addSubmapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::AddSubmap::Request> req,
    std::shared_ptr<slam_toolbox::srv::AddSubmap::Response> resp);
  void processInteractiveFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void kartoToROSOccupancyGrid(
    const karto::LocalizedRangeScanVector & scans,
    nav_msgs::srv::GetMap::Response & map);
  void transformScan(LocalizedRangeScansIt iter, tf2::Transform & submap_correction);

  // apply transformation to correct pose
  karto::Pose2 applyCorrection(const karto::Pose2 & pose, const tf2::Transform & submap_correction);
  karto::Vector2<kt_double> applyCorrection(
    const karto::Vector2<kt_double> & pose,
    const tf2::Transform & submap_correction);

  // ROS-y-ness
  std::vector<std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::MapMetaData>>> sstmS_;
  std::vector<std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>>> sstS_;
  std::shared_ptr<rclcpp::Service<slam_toolbox::srv::MergeMaps>> ssMap_;
  std::shared_ptr<rclcpp::Service<slam_toolbox::srv::AddSubmap>> ssSubmap_;

  // karto bookkeeping
  std::map<std::string, laser_utils::LaserMetadata> lasers_;
  std::vector<std::unique_ptr<karto::Dataset>> dataset_vec_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;

  // visualization
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;

  // state
  std::map<int, Eigen::Vector3d> submap_locations_;
  std::vector<karto::LocalizedRangeScanVector> scans_vec_;
  std::map<int, tf2::Transform> submap_marker_transform_;
  double resolution_;
  int num_submaps_;
};

#endif  // SLAM_TOOLBOX__MERGE_MAPS_KINEMATIC_HPP_
