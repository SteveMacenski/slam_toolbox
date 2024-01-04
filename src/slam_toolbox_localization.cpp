/*
 * slam_toolbox
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

/* Author: Steven Macenski */

#include <memory>
#include <string>
#include "slam_toolbox/slam_toolbox_localization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
LocalizationSlamToolbox::LocalizationSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
}

/*****************************************************************************/
void LocalizationSlamToolbox::loadPoseGraphByParams()
/*****************************************************************************/
{
  std::string filename;
  geometry_msgs::msg::Pose2D pose;
  bool dock = false;
  if (shouldStartWithPoseGraph(filename, pose, dock)) {
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp =
      std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Response>();
    req->initial_pose = pose;
    req->filename = filename;
    req->match_type =
      slam_toolbox::srv::DeserializePoseGraph::Request::LOCALIZE_AT_POSE;
    if (dock) {
      RCLCPP_WARN(get_logger(),
        "LocalizationSlamToolbox: Starting localization "
        "at first node (dock) is correctly not supported.");
    }

    deserializePoseGraphCallback(nullptr, req, resp);
  }
}

/*****************************************************************************/
CallbackReturn
LocalizationSlamToolbox::on_configure(const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  SlamToolbox::on_configure(state);
  processor_type_ = PROCESS_LOCALIZATION;

  // in localization mode, we cannot allow for interactive mode
  enable_interactive_mode_ = false;

  // in localization mode, disable map saver
  map_saver_.reset();
  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn
LocalizationSlamToolbox::on_activate(const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  SlamToolbox::on_activate(state);
  localization_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(
      &LocalizationSlamToolbox::localizePoseCallback,
      this, std::placeholders::_1));
  clear_localization_ = this->create_service<std_srvs::srv::Empty>(
    "slam_toolbox/clear_localization_buffer",
    std::bind(
      &LocalizationSlamToolbox::clearLocalizationBuffer, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn
LocalizationSlamToolbox::on_deactivate(const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  SlamToolbox::on_deactivate(state);
  clear_localization_.reset();
  localization_pose_sub_.reset();
  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::clearLocalizationBuffer(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> resp)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(smapper_mutex_);
  RCLCPP_INFO(get_logger(),
    "LocalizationSlamToolbox: Clearing localization buffer.");
  smapper_->clearLocalizationBuffer();
  return true;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::serializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp)
/*****************************************************************************/
{
  RCLCPP_ERROR(get_logger(), "LocalizationSlamToolbox: Cannot call serialize map "
    "in localization mode!");
  return false;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type != procType::LOCALIZE_AT_POSE) {
    RCLCPP_ERROR(get_logger(), "Requested a non-localization deserialization "
      "in localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

/*****************************************************************************/
void LocalizationSlamToolbox::laserCallback(
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
    RCLCPP_WARN(get_logger(), "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  if (shouldProcessScan(scan, pose)) {
    addScan(laser, scan, pose);
  }
}

/*****************************************************************************/
LocalizedRangeScan * LocalizationSlamToolbox::addScan(
  LaserRangeFinder * laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  Pose2 & odom_pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock l(pose_mutex_);

  if (processor_type_ == PROCESS_LOCALIZATION && process_near_pose_) {
    processor_type_ = PROCESS_NEAR_REGION;
  }

  LocalizedRangeScan * range_scan = getLocalizedRangeScan(
    laser, scan, odom_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;

  Matrix3 covariance;
  covariance.SetToIdentity();

  if (processor_type_ == PROCESS_NEAR_REGION) {
    if (!process_near_pose_) {
      RCLCPP_ERROR(get_logger(),
        "Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }

    // set our position to the requested pose and process
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan, true, &covariance);

    // reset to localization mode
    update_reprocessing_transform = true;
    processor_type_ = PROCESS_LOCALIZATION;
  } else if (processor_type_ == PROCESS_LOCALIZATION) {
    processed = smapper_->getMapper()->ProcessLocalization(range_scan, &covariance);
    update_reprocessing_transform = false;
  } else {
    RCLCPP_FATAL(get_logger(), "LocalizationSlamToolbox: "
      "No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  if (!processed) {
    delete range_scan;
    range_scan = nullptr;
  } else {
    // compute our new transform
    setTransformFromPoses(range_scan->GetCorrectedPose(), odom_pose,
      scan->header.stamp, update_reprocessing_transform);

    publishPose(range_scan->GetCorrectedPose(), covariance, scan->header.stamp);
  }

  return range_scan;
}

/*****************************************************************************/
void LocalizationSlamToolbox::localizePoseCallback(
  const
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION) {
    RCLCPP_ERROR(get_logger(),
      "LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  boost::mutex::scoped_lock l(pose_mutex_);
  if (process_near_pose_) {
    process_near_pose_.reset(new Pose2(msg->pose.pose.position.x,
      msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation)));
  } else {
    process_near_pose_ = std::make_unique<Pose2>(msg->pose.pose.position.x,
        msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
  }

  first_measurement_ = true;

  boost::mutex::scoped_lock lock(smapper_mutex_);
  smapper_->clearLocalizationBuffer();

  RCLCPP_INFO(get_logger(),
    "LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    tf2::getYaw(msg->pose.pose.orientation));
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::LocalizationSlamToolbox)
