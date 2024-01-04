/*
 * toolbox_types
 * Copyright (c) 2019, Samsung Research America
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

#ifndef SLAM_TOOLBOX__LASER_UTILS_HPP_
#define SLAM_TOOLBOX__LASER_UTILS_HPP_

#include <string>
#include <memory>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "slam_toolbox/toolbox_types.hpp"
#include "tf2/utils.h"

namespace laser_utils
{

// Convert a laser scan to a vector of readings
inline std::vector<double> scanToReadings(
  const sensor_msgs::msg::LaserScan & scan,
  const bool & inverted)
{
  std::vector<double> readings;

  if (inverted) {
    for (std::vector<float>::const_reverse_iterator it = scan.ranges.rbegin();
      it != scan.ranges.rend(); ++it)
    {
      readings.push_back(*it);
    }
  } else {
    for (std::vector<float>::const_iterator it = scan.ranges.begin(); it != scan.ranges.end();
      ++it)
    {
      readings.push_back(*it);
    }
  }

  return readings;
}

// Store laser scanner information
class LaserMetadata
{
public:
  LaserMetadata();
  ~LaserMetadata();
  LaserMetadata(karto::LaserRangeFinder * lsr, bool invert);
  bool isInverted() const;
  karto::LaserRangeFinder * getLaser();
  void invertScan(sensor_msgs::msg::LaserScan & scan) const;

private:
  karto::LaserRangeFinder * laser;
  bool inverted;
};

// Help take a scan from a laser and create a laser object
class LaserAssistant
{
public:
  template<class NodeT>
  LaserAssistant(
    NodeT node, tf2_ros::Buffer * tf,
    const std::string & base_frame);
  ~LaserAssistant();
  LaserMetadata toLaserMetadata(sensor_msgs::msg::LaserScan scan);

private:
  karto::LaserRangeFinder * makeLaser(const double & mountingYaw);
  bool isInverted(double & mountingYaw);

  rclcpp::Logger logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;
  tf2_ros::Buffer * tf_;
  sensor_msgs::msg::LaserScan scan_;
  std::string frame_, base_frame_;
  geometry_msgs::msg::TransformStamped laser_pose_;
};

// Hold some scans and utilities around them
class ScanHolder
{
public:
  explicit ScanHolder(std::map<std::string, laser_utils::LaserMetadata> & lasers);
  ~ScanHolder();
  sensor_msgs::msg::LaserScan getCorrectedScan(const int & id);
  void addScan(const sensor_msgs::msg::LaserScan scan);

private:
  std::unique_ptr<std::vector<sensor_msgs::msg::LaserScan>> current_scans_;
  std::map<std::string, laser_utils::LaserMetadata> & lasers_;
};

}  // namespace laser_utils

#endif  // SLAM_TOOLBOX__LASER_UTILS_HPP_
