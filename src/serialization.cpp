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

namespace serialization
{
void Write(const std::string& filename, const karto::LocalizedRangeScanVector& data)
{
  std::ofstream ofs(filename);
  boost::archive::text_oarchive oa(ofs);
  oa << data;
  ofs.close();
}

void Read(const std::string& filename, karto::LocalizedRangeScanVector& data)
{
  std::ifstream ifs(filename.c_str());
  boost::archive::text_iarchive ia(ifs);
  ia >> data;
}

} // end namespace