#pragma once

#include <memory>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "slam_toolbox/msg/loop_closure_event.hpp"

#include "karto_sdk/Mapper.h"

namespace slam_toolbox {


class LoopClosureListener : public karto::MapperLoopClosureListener {
public:
  LoopClosureListener(
      std::weak_ptr<rclcpp_lifecycle::LifecyclePublisher<slam_toolbox::msg::LoopClosureEvent>> loop_closure_event_pub,
      std::weak_ptr<rclcpp::Clock> clock,
      std::function<void()> loop_closure_callback);

  void EndLoopClosure(const std::string & rInfo) override;

private:
  std::weak_ptr<rclcpp_lifecycle::LifecyclePublisher<slam_toolbox::msg::LoopClosureEvent>> loop_closure_event_pub_;
  std::weak_ptr<rclcpp::Clock> clock_;
  std::function<void()> loop_closure_callback_;
};

}  // namespace slam_toolbox
