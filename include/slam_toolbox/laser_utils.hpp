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

#include <string>

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "karto_sdk/Mapper.h"

namespace tf2
{
  typedef tf2::Transform Pose;  //TODO remove
  typedef tf2::Vector3 Point;
}

namespace laser_utils
{

// Convert a laser scan to a vector of readings
inline std::vector<double> scanToReadings(const sensor_msgs::LaserScan& scan, const bool& inverted)
{
  std::vector<double> readings;

  if (inverted)
  {
    for(std::vector<float>::const_reverse_iterator it = scan.ranges.rbegin(); it != scan.ranges.rend(); ++it)
    {
      readings.push_back(*it);
    }
  }
  else 
  {
    for(std::vector<float>::const_iterator it = scan.ranges.begin(); it != scan.ranges.end(); ++it)
    {
      readings.push_back(*it);
    }
  }

  return readings;
};

// Store laser scanner information
class LaserMetadata
{
public:
  LaserMetadata()
  {
  };

  ~LaserMetadata()
  {
  }
  
  LaserMetadata(karto::LaserRangeFinder* lsr, bool invert)
  {
    laser = lsr;
    inverted = invert;
  };

  bool isInverted() const
  {
    return inverted;
  }

  karto::LaserRangeFinder* getLaser()
  {
    return laser;
  }

  void invertScan(sensor_msgs::LaserScan& scan) const
  {
    sensor_msgs::LaserScan temp;
    temp.intensities.reserve(scan.intensities.size());
    temp.ranges.reserve(scan.ranges.size());
    const bool has_intensities = scan.intensities.size() > 0 ? true : false;

    for (int i = scan.ranges.size(); i != 0; i--)
    {
      temp.ranges.push_back(scan.ranges[i]);
      if (has_intensities)
      {
        temp.intensities.push_back(scan.intensities[i]);
      }
    }

    scan.ranges = temp.ranges;
    scan.intensities = temp.intensities;
    return;
  };

private:
  karto::LaserRangeFinder* laser;
  bool inverted;
};

// Help take a scan from a laser and create a laser object
class LaserAssistant
{
public:
  LaserAssistant(ros::NodeHandle& nh, tf2_ros::Buffer* tf, const std::string& base_frame)
  : nh_(nh), tf_(tf), base_frame_(base_frame)
  {
  };

  ~LaserAssistant()
  {
  };

  LaserMetadata toLaserMetadata(sensor_msgs::LaserScan scan)
  {
    scan_ = scan;
    frame_ = scan_.header.frame_id;

    double mountingYaw;
    bool inverted = isInverted(mountingYaw);
    karto::LaserRangeFinder* laser = makeLaser(mountingYaw);
    LaserMetadata laserMeta(laser, inverted);
    return laserMeta;
  };

private:
  karto::LaserRangeFinder* makeLaser(const double & mountingYaw)
  {
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder(
      karto::LaserRangeFinder_Custom, karto::Name("Custom Described Lidar"));
    laser->SetOffsetPose(karto::Pose2(laser_pose_.transform.translation.x,
      laser_pose_.transform.translation.y, mountingYaw));
    laser->SetMinimumRange(scan_.range_min);
    laser->SetMaximumRange(scan_.range_max);
    laser->SetMinimumAngle(scan_.angle_min);
    laser->SetMaximumAngle(scan_.angle_max);
    laser->SetAngularResolution(scan_.angle_increment);

    double max_laser_range = 25;
    nh_.getParam("max_laser_range", max_laser_range);
    laser->SetRangeThreshold(max_laser_range);
    return laser;
  }

  bool isInverted(double& mountingYaw)
  {
    tf2::Stamped<tf2::Pose> ident;
    ident.setIdentity();
    ident.frame_id_ = frame_;
    ident.stamp_ = scan_.header.stamp;
    geometry_msgs::TransformStamped ident_msg;
    tf2::convert(ident, ident_msg);
    tf_->transform(ident_msg, laser_pose_, base_frame_);

    mountingYaw = tf2::getYaw(laser_pose_.transform.rotation);

    ROS_DEBUG("laser %s's pose wrt base: %.3f %.3f %.3f",
      frame_.c_str(), laser_pose_.transform.translation.x,
      laser_pose_.transform.translation.y, mountingYaw);

    tf2::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose_.transform.translation.z);
    tf2::Stamped<tf2::Vector3> up(v, scan_.header.stamp, base_frame_);
    geometry_msgs::Vector3Stamped msg;
    tf2::convert(up, msg);
    tf_->transform(msg, msg, frame_);
    
    if (up.z() <= 0)
    {
      ROS_DEBUG("laser is mounted upside-down");
      return true;
    }
    return false;

  };

  ros::NodeHandle nh_;
  tf2_ros::Buffer* tf_;
  sensor_msgs::LaserScan scan_;
  std::string frame_, base_frame_;
  geometry_msgs::TransformStamped laser_pose_;
};

} // end namespace
