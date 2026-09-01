#ifndef SLAM_TOOLBOX__INITIAL_POSE_UTILS_HPP_
#define SLAM_TOOLBOX__INITIAL_POSE_UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer_interface.h"

namespace slam_toolbox
{

inline bool transformInitialPoseToFrame(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  const std::string & target_frame,
  tf2_ros::BufferInterface & tf_buffer,
  const rclcpp::Logger & logger,
  const char * callback_name,
  geometry_msgs::msg::PoseStamped & pose_out)
{
  pose_out.header = msg.header;
  pose_out.header.frame_id = target_frame;
  pose_out.pose = msg.pose.pose;

  if (msg.header.frame_id == target_frame) {
    return true;
  }

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header = msg.header;
  pose_in.pose = msg.pose.pose;

  try {
    tf_buffer.transform(pose_in, pose_out, target_frame, tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger,
      "%s: could not transform pose from %s to %s: %s",
      callback_name,
      msg.header.frame_id.c_str(),
      target_frame.c_str(),
      ex.what());
    return false;
  }

  return true;
}

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__INITIAL_POSE_UTILS_HPP_
