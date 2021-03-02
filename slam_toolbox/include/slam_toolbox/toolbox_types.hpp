/*
 * toolbox_types
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

#ifndef SLAM_TOOLBOX_TOOLBOX_TYPES_H_
#define SLAM_TOOLBOX_TOOLBOX_TYPES_H_

#include <map>
#include <vector>

#include "tf/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "slam_toolbox/toolbox_msgs.hpp"
#include "karto_sdk/Mapper.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace toolbox_types
{

// object containing a scan pointer and a position
struct PosedScan
{
  PosedScan(sensor_msgs::LaserScan::ConstPtr scan_in, karto::Pose2 pose_in) :
             scan(scan_in), pose(pose_in) 
  {
  }
  sensor_msgs::LaserScan::ConstPtr scan;
  karto::Pose2 pose;
};

// object containing a vertex pointer and an updated score
struct ScoredVertex
{
  ScoredVertex(karto::Vertex<karto::LocalizedRangeScan>* vertex, double score)
  : vertex_(vertex), score_(score)
  {
  }

  double GetScore()
  {
    return score_;
  }

  karto::Vertex<karto::LocalizedRangeScan>* GetVertex()
  {
    return vertex_;
  }

  karto::Vertex<karto::LocalizedRangeScan>* vertex_;
  double score_;
};

typedef std::vector<ScoredVertex> ScoredVertices;
typedef std::vector<karto::Vertex<karto::LocalizedRangeScan>*> Vertices;

// types of pause functionality available
enum PausedApplication
{
  PROCESSING = 0,
  VISUALIZING_GRAPH = 1,
  NEW_MEASUREMENTS = 2
};

// types of sensor processing
enum ProcessType
{
  PROCESS = 0,
  PROCESS_FIRST_NODE = 1,
  PROCESS_NEAR_REGION = 2,
  PROCESS_LOCALIZATION = 3
};

// Pause state
struct PausedState
{
  PausedState()
  {
    state_map_[NEW_MEASUREMENTS] = false;
    state_map_[VISUALIZING_GRAPH] = false;
    state_map_[PROCESSING] = false;
  };

  void set(const PausedApplication& app, const bool& state)
  {
    boost::mutex::scoped_lock lock(pause_mutex_);
    state_map_[app] = state;
  };

  bool get(const PausedApplication& app)
  {
    boost::mutex::scoped_lock lock(pause_mutex_);
    return state_map_[app];
  };

  std::map<PausedApplication, bool> state_map_;
  boost::mutex pause_mutex_;
};

typedef std::map<karto::Name, std::map<int, karto::Vertex<karto::LocalizedRangeScan>*>> VerticeMap;
typedef std::vector<karto::Edge<karto::LocalizedRangeScan>*> EdgeVector;
typedef std::map<int, karto::Vertex<karto::LocalizedRangeScan>*> ScanMap;
typedef std::vector<karto::Vertex<karto::LocalizedRangeScan>*> ScanVector;
typedef slam_toolbox_msgs::DeserializePoseGraph::Request procType;

typedef std::unordered_map<int, Eigen::Vector3d>::iterator GraphIterator;
typedef std::unordered_map<int, Eigen::Vector3d>::const_iterator ConstGraphIterator;

}  // end namespace

#endif //SLAM_TOOLBOX_TOOLBOX_TYPES_H_
