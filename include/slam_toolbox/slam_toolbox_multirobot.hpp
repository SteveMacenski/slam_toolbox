/*
 * multirobot_slam_toolbox
 * Copyright Work Modifications (c) 2023, Achala Athukorala
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

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_

#include <memory>
#include <string>
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "slam_toolbox/toolbox_msgs.hpp"

namespace slam_toolbox
{

class MultiRobotSlamToolbox : public SlamToolbox
{
public:
  explicit MultiRobotSlamToolbox(rclcpp::NodeOptions);
  ~MultiRobotSlamToolbox() {}

protected:
  LocalizedRangeScan * addExternalScan(
    LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    Pose2 & odom_pose);
  void publishLocalizedScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    const Pose2 & offset,
    const Pose2 & pose, const Matrix3 & cov,
    const rclcpp::Time & t);

  // callbacks
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;
  void localizedScanCallback(
    slam_toolbox::msg::LocalizedLaserScan::ConstSharedPtr localized_scan);
  LaserRangeFinder * getLaser(
    const slam_toolbox::msg::LocalizedLaserScan::ConstSharedPtr localized_scan);
  using SlamToolbox::getLaser;

  std::shared_ptr<rclcpp::Publisher<slam_toolbox::msg::LocalizedLaserScan>> localized_scan_pub_;
  rclcpp::Subscription<slam_toolbox::msg::LocalizedLaserScan>::SharedPtr localized_scan_sub_;
  std::string localized_scan_topic_;
  std::string current_ns_;
};

}  // namespace slam_toolbox

#endif   // SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
