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

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_SYNC_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_SYNC_H_

#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

class SynchronousSlamToolbox : public SlamToolbox
{
public:
  SynchronousSlamToolbox(ros::NodeHandle& nh);
  ~SynchronousSlamToolbox() {};
  void run();

protected:
  virtual void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) override final;
  bool shouldProcessScan(const sensor_msgs::LaserScan::ConstPtr& scan, const karto::Pose2& pose);
  bool clearQueueCallback(slam_toolbox::ClearQueue::Request& req, slam_toolbox::ClearQueue::Response& resp);
  virtual bool deserializePoseGraphCallback(slam_toolbox::DeserializePoseGraph::Request& req,
    slam_toolbox::DeserializePoseGraph::Response& resp) override final;

  std::queue<PosedScan> q_;
  ros::ServiceServer ssClear_;
  int throttle_scans_;
  ros::Duration minimum_time_interval_;
  double minimum_travel_distance_;
};

/*****************************************************************************/
SynchronousSlamToolbox::SynchronousSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  ssClear_ = nh.advertiseService("clear_queue",
    &SynchronousSlamToolbox::clearQueueCallback, this);
  double tmp_val;
  nh.param("throttle_scans", throttle_scans_, 1);
  nh.param("minimum_time_interval", tmp_val, 0.5);
  minimum_time_interval_ = ros::Duration(tmp_val);
  minimum_travel_distance_ = smapper_->getParamMinimumTravelDistance();

  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&SynchronousSlamToolbox::run, this)));

  std::string filename;
  geometry_msgs::Pose2D pose;
  bool dock = false;
  if (shouldStartWithPoseGraph(filename, pose, dock))
  {
    slam_toolbox::DeserializePoseGraph::Request req;
    slam_toolbox::DeserializePoseGraph::Response resp;
    req.initial_pose = pose;
    req.filename = filename;
    if (dock)
    {
      req.match_type =
        slam_toolbox::DeserializePoseGraph::Request::START_AT_FIRST_NODE;
    }
    else
    {
      req.match_type =
        slam_toolbox::DeserializePoseGraph::Request::START_AT_GIVEN_POSE;      
    }
    deserializePoseGraphCallback(req, resp);
  }
}

/*****************************************************************************/
void SynchronousSlamToolbox::run()
/*****************************************************************************/
{
  ros::Rate r(100);
  while(ros::ok())
  {
    if (!q_.empty() && !isPaused(PROCESSING))
    {
      PosedScan scan_w_pose = q_.front();
      q_.pop();

      if (q_.size() > 10)
      {
        ROS_WARN_THROTTLE(10., "Queue size has grown to: %i. "
          "Recommend stopping until message is gone if online mapping.",
          (int)q_.size());
      }

      addScan(getLaser(scan_w_pose.scan), scan_w_pose);
      continue;
    }

    r.sleep();
  }
}

/*****************************************************************************/
void SynchronousSlamToolbox::laserCallback(
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

  // if sync and valid, add to queue
  if (shouldProcessScan(scan, pose))
  {
    q_.push(PosedScan(scan, pose));
  }

  return;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::shouldProcessScan(
  const sensor_msgs::LaserScan::ConstPtr& scan,
  const karto::Pose2& pose)
/*****************************************************************************/
{
  static karto::Pose2 last_pose;
  static ros::Time last_scan_time = ros::Time(0.);
  static double min_dist2 = minimum_travel_distance_ * minimum_travel_distance_;

  // we give it a pass on the first measurement to get the ball rolling
  if (first_measurement_)
  {
    last_scan_time = scan->header.stamp;
    last_pose = pose;
    first_measurement_ = false;
    return true;
  }

  // we are in a paused mode, reject incomming information
  if(isPaused(NEW_MEASUREMENTS))
  {
    return false;
  }

  // throttled out
  if ((scan->header.seq % throttle_scans_) != 0)
  {
    return false;
  }

  // not enough time
  if (scan->header.stamp - last_scan_time < minimum_time_interval_)
  {
    return false;
  }

  // check moved enough, within 10% for correction error
  const double dist2 = fabs((last_pose.GetX() - pose.GetX())*(last_pose.GetX() - 
    pose.GetX()) + (last_pose.GetY() - pose.GetY())*
    (last_pose.GetX() - pose.GetY()));
  if(dist2 < 0.8 * min_dist2 || scan->header.seq < 5)
  {
    return false;
  }

  last_pose = pose;
  last_scan_time = scan->header.stamp; 

  return true;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::clearQueueCallback(
  slam_toolbox::ClearQueue::Request& req,
  slam_toolbox::ClearQueue::Response& resp)
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
  slam_toolbox::DeserializePoseGraph::Request& req,
  slam_toolbox::DeserializePoseGraph::Response& resp)
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

  slam_toolbox::SynchronousSlamToolbox sst(nh);

  ros::spin();
  return 0;
}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_SYNC_NODE_H_