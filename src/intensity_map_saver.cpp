#include "slam_toolbox/intensity_map_saver.hpp"
#include <sstream>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"

namespace intensity_map_saver {

 
  /*****************************************************************************/
  IntensityMapSaver::IntensityMapSaver(rclcpp::Node::SharedPtr node, const std::string & map_name)
  : node_(node), map_name_(map_name), received_map_(false)
  /*****************************************************************************/
  {
    server_ = node_->create_service<slam_toolbox::srv::SaveMap>("slam_toolbox/save_intensity_map",
        std::bind(&IntensityMapSaver::saveIntensityMapCallback, this, std::placeholders::_1,
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
  bool IntensityMapSaver::saveIntensityMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> req,
    std::shared_ptr<slam_toolbox::srv::SaveMap::Response> response)
  /*****************************************************************************/
  {
    if (!received_map_) {
      RCLCPP_WARN(node_->get_logger(),
        "Intensity Map Saver: Cannot save intensity map, no map yet received on topic %s.",
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
        "SlamToolbox: Saving intensity map from topic %s as %s.", map_name_.c_str(), name.c_str());
      int rc = system(("ros2 run nav2_map_server map_saver_cli -f " + name  + " -t " + namespace_str + map_name_ + " --ros-args -p map_subscribe_transient_local:=true" + set_namespace).c_str());
      if (rc == 0) {
        response->result = response->RESULT_SUCCESS;
      } else {
        response->result = response->RESULT_UNDEFINED_FAILURE;
      }
    } else {
      RCLCPP_INFO(node_->get_logger(),
        "SlamToolbox: Saving intensity map in current directory.");
      int rc = system(("ros2 run nav2_map_server map_saver_cli -t " + namespace_str + map_name_ + " --ros-args -p map_subscribe_transient_local:=true" + set_namespace).c_str());
      if (rc == 0) {
        response->result = response->RESULT_SUCCESS;
      } else {
        response->result = response->RESULT_UNDEFINED_FAILURE;
      }
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  }

}  // namespace intensity_map_saver
