/*
 * Author
 * Copyright (c) 2018, Simbe Robotics, Inc.
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

#ifndef SLAM_TOOLBOX_SERIALIZATION_H_
#define SLAM_TOOLBOX_SERIALIZATION_H_

#include <vector>
#include <string>
#include <ros/ros.h>
#include <karto_sdk/Karto.h>
#include <karto_sdk/Mapper.h>
#include <sys/stat.h>

namespace serialization
{

inline bool fileExists(const std::string& name)
{
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

inline void write(const std::string& filename,
  karto::Mapper& mapper,
  karto::Dataset& dataset)
{
  try
  {
    mapper.SaveToFile(filename + std::string(".posegraph"));
    dataset.SaveToFile(filename + std::string(".data"));
  }
  catch (boost::archive::archive_exception e)
  {
    ROS_ERROR("Failed to write file: Exception %s", e.what());
  }
}

inline bool read(const std::string& filename,
  karto::Mapper& mapper,
  karto::Dataset& dataset)
{
  if (!fileExists(filename + std::string(".posegraph")))
  {
    ROS_ERROR("serialization::Read: Failed to open "
      "requested file: %s.", filename.c_str());
    return false;
  }

  try
  {
    mapper.LoadFromFile(filename + std::string(".posegraph"));
    dataset.LoadFromFile(filename + std::string(".data"));
    return true;
  }
  catch (boost::archive::archive_exception e)
  {
    ROS_ERROR("serialization::Read: Failed to read file: "
      "Exception: %s", e.what());
  }

  return false;
}

} // end namespace

#endif //SLAM_TOOLBOX_SERIALIZATION_H_