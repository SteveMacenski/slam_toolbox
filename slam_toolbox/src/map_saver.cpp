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

#include "slam_toolbox/map_saver.hpp"

namespace map_saver
{

/*****************************************************************************/
MapSaver::MapSaver(ros::NodeHandle & nh, const std::string& map_name)
: nh_(nh), map_name_(map_name), received_map_(false)
/*****************************************************************************/
{
  server_ = nh_.advertiseService("save_map", &MapSaver::saveMapCallback, this);
  sub_ = nh_.subscribe(map_name_, 1, &MapSaver::mapCallback, this);
}

/*****************************************************************************/
void MapSaver::mapCallback(const nav_msgs::OccupancyGrid& msg)
/*****************************************************************************/
{
  received_map_ = true;
}

/*****************************************************************************/
bool MapSaver::saveMapCallback(
  slam_toolbox_msgs::SaveMap::Request& req,
  slam_toolbox_msgs::SaveMap::Response& resp)
/*****************************************************************************/
{
  if (!received_map_)
  {
    ROS_WARN("Map Saver: Cannot save map, no map yet received on topic %s.",
      map_name_.c_str());
    return false;
  }

  const std::string name = req.name.data;
  if (name != "")
  {
    ROS_INFO("SlamToolbox: Saving map as %s.", name.c_str());
    int rc = system(("rosrun map_server map_saver -f " + name).c_str());
  }
  else
  {
    ROS_INFO("SlamToolbox: Saving map in current directory.");
    int rc = system("rosrun map_server map_saver");
  }
  ros::Duration(1.0).sleep();
  return true;
}

} // end namespace