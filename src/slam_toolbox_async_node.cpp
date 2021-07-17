/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
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

#include <memory>
#include "slam_toolbox/slam_toolbox_async.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int stack_size = 40000000;
  {
    auto temp_node = std::make_shared<rclcpp::Node>("slam_toolbox");
    temp_node->declare_parameter("stack_size_to_use",rclcpp::ParameterType::PARAMETER_INTEGER );
    if (temp_node->get_parameter("stack_size_to_use", stack_size)) {
      RCLCPP_INFO(temp_node->get_logger(), "Node using stack size %i", (int)stack_size);
      const rlim_t max_stack_size = stack_size;
      struct rlimit stack_limit;
      getrlimit(RLIMIT_STACK, &stack_limit);
      if (stack_limit.rlim_cur < stack_size) {
        stack_limit.rlim_cur = stack_size;
      }
      setrlimit(RLIMIT_STACK, &stack_limit);
    }
  }

  rclcpp::NodeOptions options;
  auto async_node = std::make_shared<slam_toolbox::AsynchronousSlamToolbox>(options);
  async_node->configure();
  async_node->loadPoseGraphByParams();
  rclcpp::spin(async_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
