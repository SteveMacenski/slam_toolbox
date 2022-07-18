/*
 * slam_toolbox
 * Copyright (c) 2022, Steve Macenski
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

#include <memory>
#include <string>
#include "slam_toolbox/experimental/slam_toolbox_map_and_localization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
MapAndLocalizationSlamToolbox::MapAndLocalizationSlamToolbox(rclcpp::NodeOptions options)
: LocalizationSlamToolbox(options)
/*****************************************************************************/
{
  // disable interactive mode
  enable_interactive_mode_ = false;

  ssSetLocalizationMode_ = create_service<std_srvs::srv::SetBool>(
    "slam_toolbox/set_localization_mode",
    std::bind(&MapAndLocalizationSlamToolbox::setLocalizationModeCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

/*****************************************************************************/
void MapAndLocalizationSlamToolbox::configure()
/*****************************************************************************/
{
  SlamToolbox::configure();
  toggleMode(false);
}

/*****************************************************************************/
bool MapAndLocalizationSlamToolbox::setLocalizationModeCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
/*****************************************************************************/
{
  toggleMode(req->data);

  resp->success = true;
  return true;
}

void MapAndLocalizationSlamToolbox::toggleMode(bool enable_localization) {
  bool in_localization_mode = processor_type_ == PROCESS_LOCALIZATION;
  if (in_localization_mode == enable_localization) {
    return;
  }

  if (enable_localization) {
    RCLCPP_INFO(get_logger(), "Enabling localization ...");
    processor_type_ = PROCESS_LOCALIZATION;

    localization_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1, std::bind(&MapAndLocalizationSlamToolbox::localizePoseCallback, this, std::placeholders::_1));
    clear_localization_ = create_service<std_srvs::srv::Empty>(
      "slam_toolbox/clear_localization_buffer",
      std::bind(&MapAndLocalizationSlamToolbox::clearLocalizationBuffer, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // in localization mode, disable map saver
    map_saver_.reset();
  }
  else {
    RCLCPP_INFO(get_logger(), "Enabling mapping ...");
    processor_type_ = PROCESS;
    localization_pose_sub_.reset();
    clear_localization_.reset();
    map_saver_ = std::make_unique<map_saver::MapSaver>(shared_from_this(), map_name_);

    boost::mutex::scoped_lock lock(smapper_mutex_);
    if (smapper_ && !smapper_->getMapper()->GetLocalizationVertices().empty()) {
      smapper_->clearLocalizationBuffer();
    }
  }
}

/*****************************************************************************/
void MapAndLocalizationSlamToolbox::loadPoseGraphByParams()
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    LocalizationSlamToolbox::loadPoseGraphByParams();
  }
  else {
    SlamToolbox::loadPoseGraphByParams();
  }
}

/*****************************************************************************/
bool MapAndLocalizationSlamToolbox::serializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    return LocalizationSlamToolbox::serializePoseGraphCallback(request_header, req, resp);
  }
  else {
    return SlamToolbox::serializePoseGraphCallback(request_header, req, resp);
  }
}

/*****************************************************************************/
bool MapAndLocalizationSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    return LocalizationSlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
  }
  else {
    return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
  }
}

/*****************************************************************************/
void MapAndLocalizationSlamToolbox::laserCallback(
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

  addScan(laser, scan, pose);
}

/*****************************************************************************/
LocalizedRangeScan * MapAndLocalizationSlamToolbox::addScan(
  LaserRangeFinder * laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  Pose2 & odom_pose)
/*****************************************************************************/
{
  if (processor_type_ == PROCESS_LOCALIZATION) {
    return LocalizationSlamToolbox::addScan(laser, scan, odom_pose);
  }
  else {
    return SlamToolbox::addScan(laser, scan, odom_pose);
  }
}

}  // namespace slam_toolbox
