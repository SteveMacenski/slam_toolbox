/*
 * slam_toolbox
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_

#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

class LocalizationSlamToolbox : public SlamToolbox
{
public:
  LocalizationSlamToolbox(ros::NodeHandle& nh);
  ~LocalizationSlamToolbox() {};

protected:
  virtual void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) override final;
  void localizePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  virtual bool serializePoseGraphCallback(slam_toolbox::SerializePoseGraph::Request& req,
    slam_toolbox::SerializePoseGraph::Response& resp) override final;

  virtual bool deserializePoseGraphCallback(slam_toolbox::DeserializePoseGraph::Request& req,
    slam_toolbox::DeserializePoseGraph::Response& resp) override final;

  ros::Subscriber localization_pose_sub_;
};

/*****************************************************************************/
LocalizationSlamToolbox::LocalizationSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  localization_pose_sub_ = node.subscribe("/initialpose", 1,
    &SlamToolbox::localizePoseCallback, this);
}

//TODO validate works
/*****************************************************************************/
bool LocalizationSlamToolbox::serializePoseGraphCallback(
  slam_toolbox::SerializePoseGraph::Request& req,
  slam_toolbox::SerializePoseGraph::Response& resp)
/*****************************************************************************/
{
  ROS_FATAL("LocalizationSlamToolbox: Cannot call serialize map "
    "in localization mode!");
  return false;
}

// TODO validate this works
/*****************************************************************************/
bool LocalizationSlamToolbox::deserializePoseGraphCallback(
  slam_toolbox::DeserializePoseGraph::Request& req,
  slam_toolbox::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
  if (req.match_type != procType::LOCALIZE_AT_POSE)
  {
    ROS_ERROR("Requested a non-localization deserialization "
      "in localization mode.");
    return false;
  }
  bool state = SlamToolbox::deserializePoseGraphCallback(req, resp);
  return state;
}

/*****************************************************************************/
void LocalizationSlamToolbox::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  // no odom info
  karto::Pose2 pose;
  if(!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // ensure the laser can be used
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  addScan(laser, scan, pose);
  return;
}

/*****************************************************************************/
void SlamToolbox::localizePoseCallback(const
  geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION)
  {
    ROS_ERROR("LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  process_near_pose_.x = msg->pose.pose.position.x;
  process_near_pose_.y = msg->pose.pose.position.y;
  process_near_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
  localization_pose_set_ = false;
  first_measurement_ = true;

  ROS_INFO("LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    process_near_pose_.x, process_near_pose_.y,
    process_near_pose_.theta);
  return;
}

} // end namespace

// program needs a larger stack size to serialize large maps
#define STACK_SIZE_TO_USE 40000000

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_toolbox");
  ros::NodeHandle nh("~");
  ros::spinOnce();

  const rlim_t max_stack_size = STACK_SIZE_TO_USE;
  struct rlimit stack_limit;
  getrlimit(RLIMIT_STACK, &stack_limit);
  if (stack_limit.rlim_cur < STACK_SIZE_TO_USE)
  {
    stack_limit.rlim_cur = STACK_SIZE_TO_USE;
  }
  setrlimit(RLIMIT_STACK, &stack_limit);

  // get initial pose, or set to XYZ and file name TODO

  slam_toolbox::LocalizationSlamToolbox sst(nh, x, y, theta);

  // localization_pose_set_ to process_near_pose_ NULL and localization mode

  ros::spin();
  return 0;
}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_
