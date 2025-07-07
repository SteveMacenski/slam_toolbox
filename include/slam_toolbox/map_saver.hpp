/*
 * map_saver
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

#ifndef SLAM_TOOLBOX__MAP_SAVER_HPP_
#define SLAM_TOOLBOX__MAP_SAVER_HPP_

#include <string>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/toolbox_msgs.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


namespace map_saver
{

// a service to save a map with a given name as requested
class MapSaver
{
public:
  MapSaver(rclcpp::Node::SharedPtr node, const std::string & topic_map_name, const std::string & topic_intensity_map_name);

protected:
  bool saveMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> request,
    std::shared_ptr<slam_toolbox::srv::SaveMap::Response> response);
  void bothMapsCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & intensity_map);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<slam_toolbox::srv::SaveMap>::SharedPtr server_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::OccupancyGrid>> map_sub_, intensity_map_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::OccupancyGrid, nav_msgs::msg::OccupancyGrid>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  nav_msgs::msg::OccupancyGrid last_map_, last_intensity_map_;
  std::string topic_map_name_, topic_intensity_map_name_;
  bool received_map_, received_intensity_map_;
};

}  // namespace map_saver

#endif  // SLAM_TOOLBOX__MAP_SAVER_HPP_
