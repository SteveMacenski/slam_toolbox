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

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "slam_toolbox/toolbox_msgs.hpp"
#include "karto_sdk/Mapper.h"

namespace pose_utils
{

// convert Karto pose to TF pose
tf::Pose kartoPose2TfPose(const karto::Pose2& pose)
{
  tf::Pose new_pose;
  new_pose.setOrigin(tf::Vector3(pose.GetX(), pose.GetY(), 0.));
  new_pose.setRotation(tf::createQuaternionFromYaw(pose.GetHeading()));
  return new_pose;
};

// helper to get the robots position
class GetPoseHelper
{
public:
  GetPoseHelper(tf::TransformListener* tf,
    const std::string& base_frame,
    const std::string& odom_frame)
  : tf_(tf), base_frame_(base_frame), odom_frame_(odom_frame)
  {
  };

  bool getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
  {
    tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0,0,0),
      tf::Vector3(0,0,0)), t, base_frame_);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
      tf_->transformPose(odom_frame_, ident, odom_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());

    karto_pose = karto::Pose2(odom_pose.getOrigin().x(),
      odom_pose.getOrigin().y(), yaw);
    return true;
  };

private:
  tf::TransformListener* tf_;
  std::string base_frame_, odom_frame_;
};

} // end namespace
