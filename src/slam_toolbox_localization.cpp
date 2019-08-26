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
  virtual void laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan) override final;
  void localizePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  virtual bool serializePoseGraphCallback(
    slam_toolbox::SerializePoseGraph::Request& req,
    slam_toolbox::SerializePoseGraph::Response& resp) override final;
  virtual bool deserializePoseGraphCallback(
    slam_toolbox::DeserializePoseGraph::Request& req,
    slam_toolbox::DeserializePoseGraph::Response& resp) override final;

  virtual bool addScan(karto::LaserRangeFinder* laser,
    const sensor_msgs::LaserScan::ConstPtr& scan,
    karto::Pose2& karto_pose) override final;

  ros::Subscriber localization_pose_sub_;
};

/*****************************************************************************/
LocalizationSlamToolbox::LocalizationSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  localization_pose_sub_ = nh.subscribe("/initialpose", 1,
    &LocalizationSlamToolbox::localizePoseCallback, this);

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
        slam_toolbox::DeserializePoseGraph::Request::LOCALIZE_AT_POSE;      
    }

    deserializePoseGraphCallback(req, resp);
  }

  return;
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
  return SlamToolbox::deserializePoseGraphCallback(req, resp);
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
bool LocalizationSlamToolbox::addScan(
  karto::LaserRangeFinder* laser,
  const sensor_msgs::LaserScan::ConstPtr& scan, 
  karto::Pose2& karto_pose)
/*****************************************************************************/
{
  if (PROCESS_LOCALIZATION && process_near_pose_)
  {
    processor_type_ = PROCESS_NEAR_REGION;
  }

  karto::LocalizedRangeScan* range_scan = getLocalizedRangeScan(
    laser, scan, karto_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false;
  if (processor_type_ == PROCESS_NEAR_REGION)
  {
    if (!process_near_pose_)
    {
      ROS_ERROR("Process near region called without a "
        "valid region request. Ignoring scan.");
      return false;
    }

    // set our position to the requested pose and process
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->ProcessAgainstNodesNearBy(range_scan);

    // compute our new transform and reset to localization mode
    setTransformFromPoses(range_scan->GetCorrectedPose(), karto_pose,
      scan->header.stamp, true);
    processor_type_ = PROCESS_LOCALIZATION;
  }
  else if (processor_type_ == PROCESS_LOCALIZATION)
  {
    processed = smapper_->ProcessLocalization(range_scan);
  }
  else
  {
    ROS_FATAL("No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  if(processed)
  {
    scan_holder_->addScan(*scan);
  }
  else
  {
    delete range_scan;
  }

  return processed;
}

/*****************************************************************************/
void LocalizationSlamToolbox::localizePoseCallback(const
  geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION)
  {
    ROS_ERROR("LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  process_near_pose_ = std::make_unique<karto::Pose2>(msg->pose.pose.position.x, 
        msg->pose.pose.position.x, tf2::getYaw(msg->pose.pose.orientation));
  first_measurement_ = true;

  ROS_INFO("LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    tf2::getYaw(msg->pose.pose.orientation));
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

  slam_toolbox::LocalizationSlamToolbox sst(nh);

  ros::spin();
  return 0;
}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_
