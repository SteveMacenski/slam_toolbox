#include "slam_toolbox/loop_closure_listener.hpp"

namespace slam_toolbox {

LoopClosureListener::LoopClosureListener(
    std::weak_ptr<rclcpp_lifecycle::LifecyclePublisher<slam_toolbox::msg::LoopClosureEvent>> loop_closure_event_pub,
    std::weak_ptr<rclcpp::Clock> clock,
    std::function<void()> loop_closure_callback)
: loop_closure_event_pub_(std::move(loop_closure_event_pub)),
  clock_(std::move(clock)),
  loop_closure_callback_(std::move(loop_closure_callback)) {}

void LoopClosureListener::EndLoopClosure(const std::string & /*rInfo*/) {
  auto spub = loop_closure_event_pub_.lock();
  auto sclk = clock_.lock();
  if (!spub || !sclk) {
    return;
  }

  slam_toolbox::msg::LoopClosureEvent event;
  event.stamp = sclk->now();
  spub->publish(event);

  if (loop_closure_callback_) {
    loop_closure_callback_();
  }
}

}  // namespace slam_toolbox
