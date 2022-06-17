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

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_ASYNC_PRUNED_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_ASYNC_PRUNED_HPP_

#include <memory>
#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

class AsynchronousSlamToolboxPruned : public SlamToolbox
{
public:
  explicit AsynchronousSlamToolboxPruned(rclcpp::NodeOptions options);
  ~AsynchronousSlamToolboxPruned() {}

protected:
  void laserCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;

  void prune(LocalizedRangeScan * range_scan);
  void removeFromSlamGraph(Vertex<LocalizedRangeScan> * vertex);


  static double computeIntersect(LocalizedRangeScan * s1, LocalizedRangeScan * s2);
  static double computeIntersectOverUnion(LocalizedRangeScan * s1, LocalizedRangeScan * s2);
  static void computeIntersectBounds(
    LocalizedRangeScan * s1, LocalizedRangeScan * s2, double & x_l,
    double & x_u, double & y_l, double & y_u);

  double iou_threshold_ = 0.85;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__SLAM_TOOLBOX_ASYNC_PRUNED_HPP_
