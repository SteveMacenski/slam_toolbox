/*
 * slam_toolbox
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

#include <map>
#include <vector>

#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace toolbox_types
{

// object containing a scan pointer and a position
struct posedScan
{
  posedScan(sensor_msgs::LaserScan::ConstPtr scan_in, karto::Pose2 pose_in) :
             scan(scan_in), pose(pose_in) 
  {
  }
  sensor_msgs::LaserScan::ConstPtr scan;
  karto::Pose2 pose;
};

// Store laser scanner information
struct laserMetadata
{
  laserMetadata()
  {
  };
  
  laserMetadata(karto::LaserRangeFinder* lsr, bool invert)
  {
    laser.reset(lsr);
    inverted = invert;
  };

  laserMetadata& operator=(laserMetadata& lmd)
  {
    //this->laser.reset(lmd.laser.release());
    laser = std::move(lmd.laser);
    this->inverted = lmd.inverted;
    return *this;
  }

  bool isInverted()
  {
    return inverted;
  }

  karto::LaserRangeFinder* getLaser()
  {
    return laser.get();
  }

  std::unique_ptr<karto::LaserRangeFinder> laser;
  bool inverted;
};

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

// convert Karto pose to TF pose
tf::Pose kartoPose2TfPose(const karto::Pose2& pose)
{
  tf::Pose new_pose;
  new_pose.setOrigin(tf::Vector3(pose.GetX(), pose.GetY(), 0.));
  new_pose.setRotation(tf::createQuaternionFromYaw(pose.GetHeading()));
  return new_pose;
};

typedef std::map<karto::Name, std::vector<karto::Vertex<karto::LocalizedRangeScan>*>> VerticeMap;
typedef std::vector<karto::Edge<karto::LocalizedRangeScan>*> EdgeVector;
typedef std::vector<karto::Vertex<karto::LocalizedRangeScan>*> ScanVector;

}  // end namespace
