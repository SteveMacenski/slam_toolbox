#ifndef SLAM_TOOLBOX_INTENSITY_MAP_SAVER_HPP
#define SLAM_TOOLBOX_INTENSITY_MAP_SAVER_HPP

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "slam_toolbox/srv/save_map.hpp"  // Reusing SaveMap service

namespace intensity_map_saver {

class IntensityMapSaver
{
public:
  IntensityMapSaver(rclcpp::Node::SharedPtr node, const std::string & service_name);
  ~IntensityMapSaver() = default;

  // Service to save the map
  bool saveIntensityMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> req,
    std::shared_ptr<slam_toolbox::srv::SaveMap::Response> response);


private:
  rclcpp::Node::SharedPtr node_;
  std::string service_name_, map_name_;
  bool received_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  // Suscriber to intensity map
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  // Server to save the map
  rclcpp::Service<slam_toolbox::srv::SaveMap>::SharedPtr server_;
};

}  // namespace intensity_map_saver

#endif  // SLAM_TOOLBOX_INTENSITY_MAP_SAVER_HPP
