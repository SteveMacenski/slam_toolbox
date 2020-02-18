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

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_

#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

class LocalizationSlamToolbox : public SlamToolbox
{
public:
  LocalizationSlamToolbox(rclcpp::NodeOptions options);
  ~LocalizationSlamToolbox() {}
  virtual void loadPoseGraphByParams();

protected:
  virtual void laserCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override final;
  void localizePoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  virtual bool serializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> resp) override final;
  virtual bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override final;

  virtual LocalizedRangeScan * addScan(
    LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    Pose2 & pose) override final;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
  localization_pose_sub_;
};

}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_LOCALIZATION_H_
