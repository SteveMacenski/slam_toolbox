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
  bool clearQueueCallback(slam_toolbox_msgs::ClearQueue::Request& req, slam_toolbox_msgs::ClearQueue::Response& resp);
  virtual bool deserializePoseGraphCallback(slam_toolbox_msgs::DeserializePoseGraph::Request& req,
    slam_toolbox_msgs::DeserializePoseGraph::Response& resp) override final;

  std::queue<PosedScan> q_;
  ros::ServiceServer ssClear_;
  boost::mutex q_mutex_;
};

}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_SYNC_NODE_H_
