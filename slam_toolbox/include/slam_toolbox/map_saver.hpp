/*
 * map_saver
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

#ifndef SLAM_TOOLBOX_MAP_SAVER_H_
#define SLAM_TOOLBOX_MAP_SAVER_H_

#include <string>
#include "ros/ros.h"
#include "slam_toolbox/toolbox_msgs.hpp"

namespace map_saver
{

// a service to save a map with a given name as requested
class MapSaver
{
public:
  MapSaver(ros::NodeHandle& nh, const std::string& service_name);

protected:
  bool saveMapCallback(slam_toolbox_msgs::SaveMap::Request& req,
                       slam_toolbox_msgs::SaveMap::Response& resp);
  void mapCallback(const nav_msgs::OccupancyGrid& msg);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  ros::Subscriber sub_;
  std::string service_name_, map_name_;
  bool received_map_;
};

} // end namespace

#endif //SLAM_TOOLBOX_MAP_SAVER_H_
