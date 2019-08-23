/*
 * Author
 * Copyright (c) 2019 Samsung Research America
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

#ifndef SLAM_TOOLBOX_MAPPER_UTILS_H_
#define SLAM_TOOLBOX_MAPPER_UTILS_H_

#include "ros/ros.h"
#include "karto_sdk/Mapper.h"
#include "tf2/utils.h"

namespace mapper_utils
{

class SMapper : public karto::Mapper
{
public:
  SMapper() : Mapper()
  {
  }

  // get occupancy grid from scans
  karto::OccupancyGrid* getOccupancyGrid(const double& resolution)
  {
    karto::OccupancyGrid* occ_grid = nullptr;
    return karto::OccupancyGrid::CreateFromScans(GetAllProcessedScans(), resolution);
  }


  // convert Karto pose to TF pose
  inline tf2::Transform toTfPose(const karto::Pose2& pose)
  {
    tf2::Transform new_pose;
    new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
    tf2::Quaternion q;
    q.setRPY(0., 0., pose.GetHeading());
    new_pose.setRotation(q);
    return new_pose;
  };

  // convert Karto pose to TF pose
  inline karto::Pose2 toKartoPose(const tf2::Transform& pose)
  {
    karto::Pose2 transformed_pose;
    transformed_pose.SetX(pose.getOrigin().x());
    transformed_pose.SetY(pose.getOrigin().y());
    transformed_pose.SetHeading(tf2::getYaw(pose.getRotation()));
    return transformed_pose;
  };

  void configure(const ros::NodeHandle& nh)
  {
    bool use_scan_matching;
    if(nh.getParam("use_scan_matching", use_scan_matching))
    {
      setParamUseScanMatching(use_scan_matching);
    }
    
    bool use_scan_barycenter;
    if(nh.getParam("use_scan_barycenter", use_scan_barycenter))
    {
      setParamUseScanBarycenter(use_scan_barycenter);
    }

    double minimum_travel_distance = 0.5;
    if(nh.getParam("minimum_travel_distance", minimum_travel_distance))
    {
      setParamMinimumTravelDistance(minimum_travel_distance);
    }

    double minimum_travel_heading;
    if(nh.getParam("minimum_travel_heading", minimum_travel_heading))
    {
      setParamMinimumTravelHeading(minimum_travel_heading);
    }

    int scan_buffer_size;
    if(nh.getParam("scan_buffer_size", scan_buffer_size))
    {
      setParamScanBufferSize(scan_buffer_size);
    }

    double scan_buffer_maximum_scan_distance;
    if(nh.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    {
      setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);
    }

    double link_match_minimum_response_fine;
    if(nh.getParam("link_match_minimum_response_fine", link_match_minimum_response_fine))
    {
      setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);
    }

    double link_scan_maximum_distance;
    if(nh.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
    {
      setParamLinkScanMaximumDistance(link_scan_maximum_distance);
    }

    double loop_search_maximum_distance;
    if(nh.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
    {
      setParamLoopSearchMaximumDistance(loop_search_maximum_distance);
    }

    bool do_loop_closing;
    if(nh.getParam("do_loop_closing", do_loop_closing))
    {
      setParamDoLoopClosing(do_loop_closing);
    }

    int loop_match_minimum_chain_size;
    if(nh.getParam("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
    {
      setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);
    }

    double loop_match_maximum_variance_coarse;
    if(nh.getParam("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
    {
      setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);
    }

    double loop_match_minimum_response_coarse;
    if(nh.getParam("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
    {
      setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);
    }

    double loop_match_minimum_response_fine;
    if(nh.getParam("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
    {
      setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);
    }

    // Setting Correlation Parameters
    double correlation_search_space_dimension;
    if(nh.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
    {
      setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);
    }

    double correlation_search_space_resolution;
    if(nh.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
    {
      setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);
    }

    double correlation_search_space_smear_deviation;
    if(nh.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    {
      setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);
    }

    // Setting Correlation Parameters, Loop Closure Parameters
    double loop_search_space_dimension;
    if(nh.getParam("loop_search_space_dimension", loop_search_space_dimension))
    {
      setParamLoopSearchSpaceDimension(loop_search_space_dimension);
    }

    double loop_search_space_resolution;
    if(nh.getParam("loop_search_space_resolution", loop_search_space_resolution))
    {
      setParamLoopSearchSpaceResolution(loop_search_space_resolution);
    }

    double loop_search_space_smear_deviation;
    if(nh.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    {
      setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);
    }

    // Setting Scan Matcher Parameters
    double distance_variance_penalty;
    if(nh.getParam("distance_variance_penalty", distance_variance_penalty))
    {
      setParamDistanceVariancePenalty(distance_variance_penalty);
    }

    double angle_variance_penalty;
    if(nh.getParam("angle_variance_penalty", angle_variance_penalty))
    {
      setParamAngleVariancePenalty(angle_variance_penalty);
    }

    double fine_search_angle_offset;
    if(nh.getParam("fine_search_angle_offset", fine_search_angle_offset))
    {
      setParamFineSearchAngleOffset(fine_search_angle_offset);
    }

    double coarse_search_angle_offset;
    if(nh.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
    {
      setParamCoarseSearchAngleOffset(coarse_search_angle_offset);
    }

    double coarse_angle_resolution;
    if(nh.getParam("coarse_angle_resolution", coarse_angle_resolution))
    {
      setParamCoarseAngleResolution(coarse_angle_resolution);
    }

    double minimum_angle_penalty;
    if(nh.getParam("minimum_angle_penalty", minimum_angle_penalty))
    {
      setParamMinimumAnglePenalty(minimum_angle_penalty);
    }

    double minimum_distance_penalty;
    if(nh.getParam("minimum_distance_penalty", minimum_distance_penalty))
    {
      setParamMinimumDistancePenalty(minimum_distance_penalty);
    }

    bool use_response_expansion;
    if(nh.getParam("use_response_expansion", use_response_expansion))
    {
      setParamUseResponseExpansion(use_response_expansion);
    }
    return;
  }
};


} // end namespace

#endif //SLAM_TOOLBOX_MAPPER_UTILS_H_
