/*
 * multirobot_slam_toolbox
 * Copyright Work Modifications (c) 2023, Achala Athukorala
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

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_

#include <memory>
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "slam_toolbox/toolbox_msgs.hpp"

namespace slam_toolbox
{

class MultiRobotSlamToolbox : public SlamToolbox
{
public:
  explicit MultiRobotSlamToolbox(rclcpp::NodeOptions);
  ~MultiRobotSlamToolbox() {};

protected:
  // callbacks
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;
};

}  // namespace slam_toolbox

#endif   // SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
