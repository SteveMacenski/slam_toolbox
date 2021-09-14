/*
 * Author
 * Copyright (c) 2019 Samsung Research America
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

#ifndef SLAM_TOOLBOX__SLAM_MAPPER_HPP_
#define SLAM_TOOLBOX__SLAM_MAPPER_HPP_

#include <memory>
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "slam_toolbox/toolbox_types.hpp"

namespace mapper_utils
{

using namespace ::karto;  // NOLINT

class SMapper
{
public:
  SMapper();
  ~SMapper();

  // get occupancy grid from scans
  karto::OccupancyGrid * getOccupancyGrid(const double & resolution);

  // convert Karto pose to TF pose
  tf2::Transform toTfPose(const karto::Pose2 & pose) const;

  // convert TF pose to karto pose
  karto::Pose2 toKartoPose(const tf2::Transform & pose) const;

  void configure(const rclcpp::Node::SharedPtr & node);
  void Reset();

  // // processors
  // kt_bool ProcessAtDock(LocalizedRangeScan* pScan);
  // kt_bool ProcessAgainstNode(LocalizedRangeScan* pScan,  const int& nodeId);
  // kt_bool ProcessAgainstNodesNearBy(LocalizedRangeScan* pScan);
  // kt_bool ProcessLocalization(LocalizedRangeScan* pScan);

  void setMapper(karto::Mapper * mapper);
  karto::Mapper * getMapper();

  void clearLocalizationBuffer();

protected:
  std::unique_ptr<karto::Mapper> mapper_;
};

}  // namespace mapper_utils

#endif   // SLAM_TOOLBOX__SLAM_MAPPER_HPP_
