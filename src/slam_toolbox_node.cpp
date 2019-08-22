/*
 * slam_toolbox
 * Copyright (c) 2018, Simbe Robotics, Inc.
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


#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_NODE_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_NODE_H_

#include <sys/resource.h>
#include "slam_toolbox_async.cpp"
#include "slam_toolbox_sync.cpp"

// program needs a larger stack size to serialize large maps
#define STACK_SIZE_TO_USE 40000000

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_toolbox");
  ros::NodeHandle nh("~");

  const rlim_t max_stack_size = STACK_SIZE_TO_USE;
  struct rlimit stack_limit;
  getrlimit(RLIMIT_STACK, &stack_limit);
  if (stack_limit.rlim_cur < STACK_SIZE_TO_USE)
  {
    stack_limit.rlim_cur = STACK_SIZE_TO_USE;
  }
  setrlimit(RLIMIT_STACK, &stack_limit);

  bool sync;
  nh.param("synchronous", sync, true);
  if (sync)
  {
    slam_toolbox::SynchronousSlamToolbox sst(nh);
  }
  else
  {
    slam_toolbox::AsynchronousSlamToolbox ast(nh);
  }

  // TODO if localization mode, start with given position, or 0,0,0 with file, or exit no file

  ros::spin();
  return 0;
}

#endif // SLAM_TOOLBOX_SLAM_TOOLBOX_NODE_H_
