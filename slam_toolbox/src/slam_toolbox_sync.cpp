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

namespace slam_toolbox
{

/*****************************************************************************/
SynchronousSlamToolbox::SynchronousSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  ssClear_ = nh.advertiseService("clear_queue",
    &SynchronousSlamToolbox::clearQueueCallback, this);

  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&SynchronousSlamToolbox::run, this)));

  loadPoseGraphByParams(nh);
}

/*****************************************************************************/
void SynchronousSlamToolbox::run()
/*****************************************************************************/
{
  ros::Rate r(100);
  while(ros::ok())
  {
    if (!isPaused(PROCESSING))
    {
      PosedScan scan_w_pose(nullptr, karto::Pose2()); // dummy, updated in critical section
      bool queue_empty = true;
      {
        boost::mutex::scoped_lock lock(q_mutex_);
        queue_empty = q_.empty();
        if(!queue_empty)
        {
          scan_w_pose = q_.front();
          q_.pop();

          if (q_.size() > 10)
          {
            ROS_WARN_THROTTLE(10., "Queue size has grown to: %i. "
              "Recommend stopping until message is gone if online mapping.",
              (int)q_.size());
          }
        }
      }
      if(!queue_empty){
        addScan(getLaser(scan_w_pose.scan), scan_w_pose);
        continue;
      }
    }

    r.sleep();
  }
}

/*****************************************************************************/
void SynchronousSlamToolbox::laserCallback(
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
    ROS_WARN_THROTTLE(5., "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // if sync and valid, add to queue
  if (shouldProcessScan(scan, pose))
  {
    boost::mutex::scoped_lock lock(q_mutex_);
    q_.push(PosedScan(scan, pose));
  }

  return;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::clearQueueCallback(
  slam_toolbox_msgs::ClearQueue::Request& req,
  slam_toolbox_msgs::ClearQueue::Response& resp)
/*****************************************************************************/
{
  ROS_INFO("SynchronousSlamToolbox: Clearing all queued scans to add to map.");
  while(!q_.empty())
  {
    q_.pop();
  }
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::deserializePoseGraphCallback(
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
