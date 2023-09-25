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

#include <memory>
#include <string>
#include "slam_toolbox/map_saver.hpp"

namespace map_saver
{

/*****************************************************************************/
MapSaver::MapSaver(rclcpp::Node::SharedPtr node, const std::string & map_name)
: node_(node), map_name_(map_name), received_map_(false)
/*****************************************************************************/
{
  server_ = node_->create_service<slam_toolbox::srv::SaveMap>("slam_toolbox/save_map",
      std::bind(&MapSaver::saveMapCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  auto mapCallback =
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void
    {
      received_map_ = true;
    };

  sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_name_, rclcpp::QoS(1), mapCallback);
}

/*****************************************************************************/
bool MapSaver::saveMapCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> req,
  std::shared_ptr<slam_toolbox::srv::SaveMap::Response> response)
/*****************************************************************************/
{
  if (!received_map_) {
    RCLCPP_WARN(node_->get_logger(),
      "Map Saver: Cannot save map, no map yet received on topic %s.",
      map_name_.c_str());
    response->result = response->RESULT_NO_MAP_RECEIEVD;
    return false;
  }

  const std::string name = req->name.data;
  std::string set_namespace;
  const std::string namespace_str = std::string(node_->get_namespace());
  if (!namespace_str.empty()) {
    set_namespace = " -r __ns:=" + namespace_str;
  }

  if (name != "") {
    RCLCPP_INFO(node_->get_logger(),
      "SlamToolbox: Saving map as %s.", name.c_str());
    int rc = system(("ros2 run nav2_map_server map_saver_cli -f " + name  + " --ros-args -p map_subscribe_transient_local:=true" + set_namespace).c_str());
    if (rc == 0) {
      response->result = response->RESULT_SUCCESS;
    } else {
      response->result = response->RESULT_UNDEFINED_FAILURE;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
      "SlamToolbox: Saving map in current directory.");
    int rc = system(("ros2 run nav2_map_server map_saver_cli --ros-args -p map_subscribe_transient_local:=true" + set_namespace).c_str());
    if (rc == 0) {
      response->result = response->RESULT_SUCCESS;
    } else {
      response->result = response->RESULT_UNDEFINED_FAILURE;
    }
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

}  // namespace map_saver
