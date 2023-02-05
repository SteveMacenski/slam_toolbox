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

#include "slam_toolbox/slam_toolbox_async.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
AsynchronousSlamToolbox::AsynchronousSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  loadPoseGraphByParams(nh);
}

/*****************************************************************************/
void AsynchronousSlamToolbox::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan, const std::string& base_frame_id)
/*****************************************************************************/
{
  // no odom info on any pose helper
  karto::Pose2 pose;
  if(pose_helpers_.find(base_frame_id) == pose_helpers_.end())
    return;
  else
  {
    pose_helpers_[base_frame_id]->getOdomPose(pose, scan->header.stamp, base_frame_id);
  }

  // Add new sensor to laser ID map, and to laser assistants
  {
    boost::mutex::scoped_lock l(laser_id_map_mutex_);
    if(m_laser_id_to_base_id_.find(scan->header.frame_id) == m_laser_id_to_base_id_.end())
    {
      m_laser_id_to_base_id_[scan->header.frame_id] = base_frame_id;
      laser_assistants_[scan->header.frame_id] = std::make_unique<laser_utils::LaserAssistant>(nh_, tf_.get(), base_frame_id);
    }
  }
  // ensure the laser can be used
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  addScan(laser, scan, pose);
  return;
}

/*****************************************************************************/
bool AsynchronousSlamToolbox::deserializePoseGraphCallback(
  slam_toolbox_msgs::DeserializePoseGraph::Request& req,
  slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
  if (req.match_type == procType::LOCALIZE_AT_POSE)
  {
    ROS_ERROR("Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

} // end namespace
