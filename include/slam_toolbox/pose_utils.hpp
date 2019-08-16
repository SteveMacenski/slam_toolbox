/*
 * snap_utils
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

#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "slam_toolbox/toolbox_msgs.hpp"
#include "karto_sdk/Mapper.h"

namespace tf2
{
  typedef tf2::Transform Pose;  //TODO remove
  typedef tf2::Vector3 Point;
}

namespace pose_utils
{

// convert Karto pose to TF pose
tf2::Pose kartoPose2TfPose(const karto::Pose2& pose)
{
  tf2::Pose new_pose;
  new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  tf2::Quaternion q;
  q.setEuler(pose.GetHeading(), 0., 0.);
  new_pose.setRotation(q);
  return new_pose;
};

// helper to get the robots position
class GetPoseHelper
{
public:
  GetPoseHelper(tf2_ros::Buffer* tf,
    const std::string& base_frame,
    const std::string& odom_frame)
  : tf_(tf), base_frame_(base_frame), odom_frame_(odom_frame)
  {
  };

  bool getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
  {
    tf2::Stamped<tf2::Pose> ident(tf2::Transform(tf2::Quaternion(),
      tf2::Vector3(0,0,0)), t, base_frame_);
    tf2::Stamped<tf2::Transform> odom_pose;

    geometry_msgs::TransformStamped ident_msg, odom_pose_msg;
    tf2::convert(ident, ident_msg);
    tf2::convert(odom_pose, odom_pose_msg);

    try
    {
      tf_->transform(ident_msg, odom_pose_msg, odom_frame_);
    }
    catch(tf2::TransformException e)
    {
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      return false;
    }
    double yaw = tf2::getYaw(odom_pose.getRotation());

    karto_pose = karto::Pose2(odom_pose.getOrigin().x(),
      odom_pose.getOrigin().y(), yaw);
    return true;
  };

private:
  tf2_ros::Buffer* tf_;
  std::string base_frame_, odom_frame_;
};

} // end namespace
