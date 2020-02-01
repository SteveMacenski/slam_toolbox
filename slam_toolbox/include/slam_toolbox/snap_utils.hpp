/*
 * snap_utils
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

#ifndef SLAM_TOOLBOX_SNAP_UTILS_H_
#define SLAM_TOOLBOX_SNAP_UTILS_H_

namespace snap_utils
{

// whether this is running in a snap container
inline bool isInSnap()
{
  char* snap_common = getenv("SNAP_COMMON");
  if (snap_common != NULL)
  {
    return true;
  }
  return false;
};

// get path of shared space
inline std::string getSnapPath()
{
  char* snap_common = getenv("SNAP_COMMON");
  return std::string(snap_common);
}

}  // end namespace

#endif //SLAM_TOOLBOX_SNAP_UTILS_H_
