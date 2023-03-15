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

#include "slam_toolbox/slam_toolbox_multirobot.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
MultiRobotSlamToolbox::MultiRobotSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options), localized_scan_topic_("/localized_scan")
/*****************************************************************************/
{
    current_ns_ = this->get_namespace() + 1;

    localized_scan_pub_ = this->create_publisher<slam_toolbox::msg::LocalizedLaserScan>(
    localized_scan_topic_, 10);
}

/*****************************************************************************/
void MultiRobotSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(get_logger(), "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  LocalizedRangeScan * range_scan = addScan(laser, scan, pose);
  if (range_scan != nullptr)
  {
    Matrix3 covariance;
    covariance.SetToIdentity();
    publishLocalizedScan(scan, laser->GetOffsetPose(), 
        range_scan->GetCorrectedPose(), covariance, scan->header.stamp);
  }
}


/*****************************************************************************/
void MultiRobotSlamToolbox::publishLocalizedScan( 
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  const Pose2 & offset,
  const Pose2 & pose,
  const Matrix3 & cov,
  const rclcpp::Time & t)
/*****************************************************************************/
{
  slam_toolbox::msg::LocalizedLaserScan scan_msg; 

  scan_msg.scan = *scan;

  tf2::Quaternion q_offset(0., 0., 0., 1.0);
  q_offset.setRPY(0., 0., offset.GetHeading());
  tf2::Transform scanner_offset(q_offset, tf2::Vector3(offset.GetX(), offset.GetY(), 0.0));
  tf2::toMsg(scanner_offset, scan_msg.scanner_offset.transform);
  scan_msg.scanner_offset.header.stamp = t;

  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., pose.GetHeading());
  tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
  tf2::toMsg(transform, scan_msg.pose.pose.pose);

  scan_msg.pose.pose.covariance[0] = cov(0, 0) * position_covariance_scale_;  // x
  scan_msg.pose.pose.covariance[1] = cov(0, 1) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[6] = cov(1, 0) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[7] = cov(1, 1) * position_covariance_scale_;  // y
  scan_msg.pose.pose.covariance[35] = cov(2, 2) * yaw_covariance_scale_;      // yaw
  scan_msg.pose.header.stamp = t;

  // Set prefixed frame names
  scan_msg.scan.header.frame_id = (*(scan->header.frame_id.cbegin()) == '/') ? 
    current_ns_ + scan->header.frame_id :
    current_ns_ + "/" + scan->header.frame_id;

  scan_msg.pose.header.frame_id = (*(map_frame_.cbegin()) == '/') ? 
    current_ns_ + map_frame_ :
    current_ns_ + "/" + map_frame_;

  scan_msg.scanner_offset.child_frame_id = scan_msg.scan.header.frame_id;

  scan_msg.scanner_offset.header.frame_id = (*(base_frame_.cbegin()) == '/') ?
    current_ns_ + base_frame_ :
    current_ns_ + "/" + base_frame_;

  localized_scan_pub_->publish(scan_msg);
}

/*****************************************************************************/
bool MultiRobotSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_WARN(get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

}