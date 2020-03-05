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

namespace map_saver
{

// a service to save a map with a given name as requested
class MapSaver
{
public:
  MapSaver(rclcpp::Node::SharedPtr node, const std::string & service_name);

protected:
  bool saveMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> request,
    std::shared_ptr<slam_toolbox::srv::SaveMap::Response> response);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<slam_toolbox::srv::SaveMap>::SharedPtr server_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::string service_name_, map_name_;
  bool received_map_;
};

}  // namespace map_saver

#endif  // SLAM_TOOLBOX__MAP_SAVER_HPP_
