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

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_SYNC_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_SYNC_HPP_

#include <queue>
#include <memory>
#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

class SynchronousSlamToolbox : public SlamToolbox
{
public:
  explicit SynchronousSlamToolbox(rclcpp::NodeOptions options);
  ~SynchronousSlamToolbox() {}
  void run();

protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
  bool clearQueueCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::ClearQueue::Request> req,
    std::shared_ptr<slam_toolbox::srv::ClearQueue::Response> resp);
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;

  std::queue<PosedScan> q_;
  std::shared_ptr<rclcpp::Service<slam_toolbox::srv::ClearQueue>> ssClear_;
  boost::mutex q_mutex_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__SLAM_TOOLBOX_SYNC_HPP_
