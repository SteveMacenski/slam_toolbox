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

}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_
