/*
 * system_utils
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

#ifndef SLAM_TOOLBOX__SYSTEM_UTILS_HPP_
#define SLAM_TOOLBOX__SYSTEM_UTILS_HPP_

#ifndef _WIN32
  #include <sys/resource.h>
#endif

namespace system_utils
{

inline void setStackLimitMaximum(size_t stack_size)
{
#ifndef _WIN32
  const rlim_t max_stack_size = stack_size;
  struct rlimit stack_limit;
  getrlimit(RLIMIT_STACK, &stack_limit);
  if (stack_limit.rlim_cur < stack_size) {
    stack_limit.rlim_cur = stack_size;
  }
  setrlimit(RLIMIT_STACK, &stack_limit);
#endif
}

}  // namespace system_utils

#endif  // SLAM_TOOLBOX__SYSTEM_UTILS_HPP_
