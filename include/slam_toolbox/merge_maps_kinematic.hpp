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

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h> 
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

#include "open_karto/Mapper.h"

#include <boost/thread.hpp>

#include "slam_toolbox/MergeMaps.h"
#include "slam_toolbox/AddSubmap.h"

#include <string>
#include <map>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class MergeMapTool
{

public:
  MergeMapTool();
  ~MergeMapTool();

private:
  typedef std::unique_ptr<karto::Pose2> Pose2_ptr;
  typedef std::unique_ptr<karto::Vector2<kt_double>> Vector2_double_ptr;
  // setup
  void SetConfigs();

  // callbacks
  bool MergeMapCallback(slam_toolbox::MergeMaps::Request  &req, slam_toolbox::MergeMaps::Response &resp);
  bool AddSubmapCallback(slam_toolbox::AddSubmap::Request  &req, slam_toolbox::AddSubmap::Response &resp);
  void ProcessInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void KartoToROSOccupancyGrid(const karto::LocalizedRangeScanVector& scans, nav_msgs::GetMap::Response& map);

  Pose2_ptr ApplyCorrection(const karto::Pose2& pose, const tf::Transform& submap_correction)
  {
    tf::Transform pose_tf, pose_corr;
    pose_tf.setOrigin(tf::Vector3(pose.GetX(), pose.GetY(), 0.));
    pose_tf.setRotation(tf::createQuaternionFromRPY(0, 0, pose.GetHeading()));
    pose_corr = submap_correction*pose_tf;
    return Pose2_ptr(new karto::Pose2(pose_corr.getOrigin().x(), pose_corr.getOrigin().y(), tf::getYaw(pose_corr.getRotation())));
  }

  Vector2_double_ptr ApplyCorrection(const karto::Vector2<kt_double>&  pose, const tf::Transform& submap_correction)
  {
    tf::Transform pose_tf, pose_corr;
    pose_tf.setOrigin(tf::Vector3(pose.GetX(), pose.GetY(), 0.));
    pose_tf.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    pose_corr = submap_correction*pose_tf;
    return Vector2_double_ptr(new karto::Vector2<kt_double>(pose_corr.getOrigin().x(), pose_corr.getOrigin().y()));
  }

  inline bool FileExists(const std::string& name)
  {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0); 
  }

  typedef std::vector<karto::LocalizedRangeScanVector>::iterator LocalizedRangeScansVec_it;
  typedef karto::LocalizedRangeScanVector::iterator LocalizedRangeScans_it;

  // ROS-y-ness
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> sstS_, sstmS_;
  ros::ServiceServer ssMap_, ssSubmap_;

  // param and state
  std::string map_frame_;
  double resolution_;
  double max_laser_range_;
  int num_submaps_;

  //karto bookkeeping
  std::map<std::string, karto::LaserRangeFinder*> lasers_;

  // TF
  tf::TransformBroadcaster* tfB_;

  // visualization
  interactive_markers::InteractiveMarkerServer* interactive_server_;
  std::map<int, Eigen::Vector3d> submap_locations_;
  std::vector<karto::LocalizedRangeScanVector> scans_vec_;
  std::map<int, tf::Transform> submap_marker_transform_;
};
