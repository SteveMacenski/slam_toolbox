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

#include <vector>
#include <string>
#include <ros/ros.h>
#include <open_karto/Karto.h>
#include <open_karto/Mapper.h>
#include <sys/stat.h>



inline bool FileExists(const std::string& name)
{
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

namespace serialization
{

void Write(const std::string& filename, karto::Mapper* mapper, karto::Dataset* dataset)
{
  try
  {
    mapper->SaveToFile(filename + std::string(".st"));
  }
  catch (boost::archive::archive_exception e)
  {
    ROS_ERROR("Failed to write file: Exception %s", e.what());
  }
  std::ofstream ofs((filename + std::string(".data")).c_str());
  boost::archive::binary_oarchive oa(ofs, boost::archive::no_codecvt);
  oa << BOOST_SERIALIZATION_NVP(dataset);
}

void Read(const std::string& filename, karto::Mapper* mapper, karto::Dataset*& dataset)
{
  if (!FileExists(filename + std::string(".st")))
  {
    ROS_ERROR("serialization::Read : Failed to open requested submap %s.", filename.c_str());
  }
  try
  {
    mapper->LoadFromFile(filename + std::string(".st"));
  }
  catch (boost::archive::archive_exception e)
  {
    ROS_ERROR("Failed to read file: Exception %s", e.what());
  }
  std::ifstream ifs((filename + std::string(".data")).c_str());
  boost::archive::binary_iarchive ia(ifs);
  ia >> BOOST_SERIALIZATION_NVP(dataset);
}

} // end namespace
