/*
 * slam_toolbox
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

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_LOCALIZATION_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_LOCALIZATION_HPP_

#include <memory>
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "std_srvs/srv/empty.hpp"

namespace slam_toolbox
{

class LocalizationSlamToolbox : public SlamToolbox
{
public:
  explicit LocalizationSlamToolbox(rclcpp::NodeOptions options);
  virtual ~LocalizationSlamToolbox() {}
  virtual void loadPoseGraphByParams();
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  virtual void laserCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
  void localizePoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool clearLocalizationBuffer(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> resp);

  virtual bool serializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp) override;
  virtual bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;

  virtual LocalizedRangeScan * addScan(
    LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    Pose2 & pose) override;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
  localization_pose_sub_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty> > clear_localization_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__SLAM_TOOLBOX_LOCALIZATION_HPP_
