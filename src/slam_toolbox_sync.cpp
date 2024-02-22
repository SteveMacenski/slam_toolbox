/*
 * slam_toolbox
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

/* Author: Steven Macenski */

#include "slam_toolbox/slam_toolbox_sync.hpp"

#include <memory>
namespace slam_toolbox
{

/*****************************************************************************/
SynchronousSlamToolbox::SynchronousSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
}

/*****************************************************************************/
void SynchronousSlamToolbox::run()
/*****************************************************************************/
{
  rclcpp::Rate r(100);
  while (rclcpp::ok()) {
    boost::this_thread::interruption_point();
    if (!isPaused(PROCESSING)) {
      PosedScan scan_w_pose(nullptr, karto::Pose2()); // dummy, updated in critical section
      bool queue_empty = true;
      {
        boost::mutex::scoped_lock lock(q_mutex_);
        queue_empty = q_.empty();
        if (!queue_empty) {
          scan_w_pose = q_.front();
          q_.pop();

          if (q_.size() > 10) {
            RCLCPP_WARN(get_logger(), "Queue size has grown to: %i. "
              "Recommend stopping until message is gone if online mapping.",
              (int)q_.size());
          }
        }
      }
      if (!queue_empty) {
        addScan(getLaser(scan_w_pose.scan), scan_w_pose);
        continue;
      }
    }

    r.sleep();
  }
}

/*****************************************************************************/
CallbackReturn
SynchronousSlamToolbox::on_activate(const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  SlamToolbox::on_activate(state);
  ssClear_ = this->create_service<slam_toolbox::srv::ClearQueue>(
    "slam_toolbox/clear_queue",
    std::bind(
      &SynchronousSlamToolbox::clearQueueCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  threads_.push_back(
    std::make_unique<boost::thread>(
      boost::bind(&SynchronousSlamToolbox::run, this)));
  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn
SynchronousSlamToolbox::on_deactivate(const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  SlamToolbox::on_deactivate(state);
  ssClear_.reset();
  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
void SynchronousSlamToolbox::laserCallback(
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

  // if sync and valid, add to queue
  if (shouldProcessScan(scan, pose)) {
    boost::mutex::scoped_lock lock(q_mutex_);
    q_.push(PosedScan(scan, pose));
  }
}

/*****************************************************************************/
bool SynchronousSlamToolbox::clearQueueCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::ClearQueue::Request> req,
  std::shared_ptr<slam_toolbox::srv::ClearQueue::Response> resp)
/*****************************************************************************/
{
  RCLCPP_INFO(get_logger(), "SynchronousSlamToolbox: "
    "Clearing all queued scans to add to map.");
  while (!q_.empty()) {
    q_.pop();
  }
  resp->status = true;
  return resp->status;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_ERROR(get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

/*****************************************************************************/
bool SynchronousSlamToolbox::resetCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::Reset::Request> req,
  std::shared_ptr<slam_toolbox::srv::Reset::Response> resp)
/*****************************************************************************/
{
  {
    boost::mutex::scoped_lock lock(q_mutex_);
    // Clear the scan queue.
    while (!q_.empty()) {
      q_.pop();
    }
  }
  return SlamToolbox::resetCallback(request_header, req, resp);
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::SynchronousSlamToolbox)
