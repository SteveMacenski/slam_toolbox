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
#include <rmw/types.h>

namespace map_saver
{

/*****************************************************************************/
MapSaver::MapSaver(rclcpp::Node::SharedPtr node, const std::string & topic_map_name, const std::string & topic_intensity_map_name)
: node_(node), topic_map_name_(topic_map_name), topic_intensity_map_name_(topic_intensity_map_name), received_map_(false), received_intensity_map_(false)
/*****************************************************************************/
{
  server_ = node_->create_service<slam_toolbox::srv::SaveMap>("slam_toolbox/save_map",
      std::bind(&MapSaver::saveMapCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  map_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::OccupancyGrid>>(
    node_.get(), topic_map_name_, rmw_qos_profile_sensor_data);

  intensity_map_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::OccupancyGrid>>(
    node_.get(), topic_intensity_map_name_, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *map_sub_, *intensity_map_sub_);
  sync_->registerCallback(
      std::bind(&MapSaver::bothMapsCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  if (!node_->has_parameter("map_types_to_save")) {
    node_->declare_parameter<std::string>("map_types_to_save", "both");
  }
}

/*****************************************************************************/
void MapSaver::bothMapsCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map,
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & intensity_map)
/*****************************************************************************/
{
  last_map_ = *map;
  last_intensity_map_ = *intensity_map;
  received_map_ = true;
  received_intensity_map_ = true;
}

/*****************************************************************************/
bool MapSaver::saveMapCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> req,
  std::shared_ptr<slam_toolbox::srv::SaveMap::Response> response)
/*****************************************************************************/
{
  std::string map_type = "both";
  node_->get_parameter("map_types_to_save", map_type);

  if (map_type == "occupancy" || map_type == "both") {
    if (!received_map_) {
      RCLCPP_WARN(node_->get_logger(),
        "Map Saver: Cannot save map, no map yet received on topic %s.",
        topic_map_name_.c_str());
      response->result = response->RESULT_NO_MAP_RECEIEVD;
      return false;
    }
  }
  if (map_type == "intensity" || map_type == "both") {
      if (!received_intensity_map_) {
        RCLCPP_WARN(node_->get_logger(),
          "Map Saver: Cannot save map, no map yet received on topic %s.",
          topic_intensity_map_name_.c_str());
        response->result = response->RESULT_NO_MAP_RECEIEVD;
        return false;
      } 
  }

  const std::string file_map_name = req->name.data.empty() ? "map" : req->name.data;
  const std::string namespace_str = std::string(node_->get_namespace());
  auto make_cmd = [&](const std::string &topic, const std::string &file) {
    std::string cmd = "ros2 run nav2_map_server map_saver_cli -f " + file +
                      " -t " + (namespace_str == "/" ? "" : namespace_str) + topic +
                      " --ros-args -p map_subscribe_transient_local:=true";
    return cmd;
  };

  bool success_occ = false, success_int = false;

  if (map_type == "occupancy" || map_type == "both") { 
    std::string cmd = make_cmd(topic_map_name_, file_map_name);         
    success_occ = (system(cmd.c_str()) == 0);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (map_type == "intensity" || map_type == "both") {
    std::string cmd = make_cmd(topic_intensity_map_name_, (file_map_name +"_intensity"));      
    success_int = (system(cmd.c_str()) == 0);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  bool result = ((map_type == "both" && success_occ && success_int) ||
                 (map_type == "occupancy" && success_occ) ||
                 (map_type == "intensity" && success_int));

  response->result = result ? response->RESULT_SUCCESS : response->RESULT_UNDEFINED_FAILURE;

  return false;
}

}  // namespace map_saver
