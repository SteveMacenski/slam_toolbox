/*
 * slam_mapper
 * Copyright (c) 2018, Simbe Robotics
 * Copyright (c) 2018, Steve Macenski
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

#include "slam_toolbox/slam_mapper.hpp"

namespace mapper_utils
{

/*****************************************************************************/
SMapper::SMapper()
/*****************************************************************************/
{
  mapper_ = std::make_unique<karto::Mapper>(); 
}

/*****************************************************************************/
SMapper::~SMapper()
/*****************************************************************************/
{
  mapper_.reset();
}

/*****************************************************************************/
karto::Mapper* SMapper::getMapper()
/*****************************************************************************/
{
  return mapper_.get();
}

/*****************************************************************************/
void SMapper::setMapper(karto::Mapper* mapper)
/*****************************************************************************/
{
  mapper_.reset(mapper);
}

/*****************************************************************************/
void SMapper::clearLocalizationBuffer()
/*****************************************************************************/
{
  mapper_->ClearLocalizationBuffer();
}

/*****************************************************************************/
karto::OccupancyGrid* SMapper::getOccupancyGrid(const double& resolution)
/*****************************************************************************/
{
  karto::OccupancyGrid* occ_grid = nullptr;
  return karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(),
    resolution);
}

/*****************************************************************************/
tf2::Transform SMapper::toTfPose(const karto::Pose2& pose) const
/*****************************************************************************/
{
  tf2::Transform new_pose;
  new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  tf2::Quaternion q;
  q.setRPY(0., 0., pose.GetHeading());
  new_pose.setRotation(q);
  return new_pose;
};

/*****************************************************************************/
karto::Pose2 SMapper::toKartoPose(const tf2::Transform& pose) const
/*****************************************************************************/
{
  karto::Pose2 transformed_pose;
  transformed_pose.SetX(pose.getOrigin().x());
  transformed_pose.SetY(pose.getOrigin().y());
  transformed_pose.SetHeading(tf2::getYaw(pose.getRotation()));
  return transformed_pose;
};

/*****************************************************************************/
void SMapper::configure(const ros::NodeHandle& nh)
/*****************************************************************************/
{
  bool use_scan_matching;
  if(nh.getParam("use_scan_matching", use_scan_matching))
  {
    mapper_->setParamUseScanMatching(use_scan_matching);
  }
  
  bool use_scan_barycenter;
  if(nh.getParam("use_scan_barycenter", use_scan_barycenter))
  {
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);
  }

  double minimum_travel_distance = 0.5;
  if(nh.getParam("minimum_travel_distance", minimum_travel_distance))
  {
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);
  }

  double minimum_travel_heading;
  if(nh.getParam("minimum_travel_heading", minimum_travel_heading))
  {
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);
  }

  int scan_buffer_size;
  if(nh.getParam("scan_buffer_size", scan_buffer_size))
  {
    mapper_->setParamScanBufferSize(scan_buffer_size);
  }

  double scan_buffer_maximum_scan_distance;
  if(nh.getParam("scan_buffer_maximum_scan_distance",
    scan_buffer_maximum_scan_distance))
  {
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);
  }

  double link_match_minimum_response_fine;
  if(nh.getParam("link_match_minimum_response_fine",
    link_match_minimum_response_fine))
  {
    mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);
  }

  double link_scan_maximum_distance;
  if(nh.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
  {
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);
  }

  double loop_search_maximum_distance;
  if(nh.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
  {
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);
  }

  bool do_loop_closing;
  if(nh.getParam("do_loop_closing", do_loop_closing))
  {
    mapper_->setParamDoLoopClosing(do_loop_closing);
  }

  int loop_match_minimum_chain_size;
  if(nh.getParam("loop_match_minimum_chain_size",
    loop_match_minimum_chain_size))
  {
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);
  }

  double loop_match_maximum_variance_coarse;
  if(nh.getParam("loop_match_maximum_variance_coarse",
    loop_match_maximum_variance_coarse))
  {
    mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);
  }

  double loop_match_minimum_response_coarse;
  if(nh.getParam("loop_match_minimum_response_coarse",
    loop_match_minimum_response_coarse))
  {
    mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);
  }

  double loop_match_minimum_response_fine;
  if(nh.getParam("loop_match_minimum_response_fine",
    loop_match_minimum_response_fine))
  {
    mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);
  }

  // Setting Correlation Parameters
  double correlation_search_space_dimension;
  if(nh.getParam("correlation_search_space_dimension",
    correlation_search_space_dimension))
  {
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);
  }

  double correlation_search_space_resolution;
  if(nh.getParam("correlation_search_space_resolution",
    correlation_search_space_resolution))
  {
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);
  }

  double correlation_search_space_smear_deviation;
  if(nh.getParam("correlation_search_space_smear_deviation",
    correlation_search_space_smear_deviation))
  {
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(
      correlation_search_space_smear_deviation);
  }

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension;
  if(nh.getParam("loop_search_space_dimension", loop_search_space_dimension))
  {
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);
  }

  double loop_search_space_resolution;
  if(nh.getParam("loop_search_space_resolution", loop_search_space_resolution))
  {
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);
  }

  double loop_search_space_smear_deviation;
  if(nh.getParam("loop_search_space_smear_deviation",
    loop_search_space_smear_deviation))
  {
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);
  }

  // Setting Scan Matcher Parameters
  double distance_variance_penalty;
  if(nh.getParam("distance_variance_penalty", distance_variance_penalty))
  {
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);
  }

  double angle_variance_penalty;
  if(nh.getParam("angle_variance_penalty", angle_variance_penalty))
  {
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);
  }

  double fine_search_angle_offset;
  if(nh.getParam("fine_search_angle_offset", fine_search_angle_offset))
  {
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);
  }

  double coarse_search_angle_offset;
  if(nh.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
  {
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);
  }

  double coarse_angle_resolution;
  if(nh.getParam("coarse_angle_resolution", coarse_angle_resolution))
  {
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);
  }

  double minimum_angle_penalty;
  if(nh.getParam("minimum_angle_penalty", minimum_angle_penalty))
  {
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);
  }

  double minimum_distance_penalty;
  if(nh.getParam("minimum_distance_penalty", minimum_distance_penalty))
  {
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);
  }

  bool use_response_expansion;
  if(nh.getParam("use_response_expansion", use_response_expansion))
  {
    mapper_->setParamUseResponseExpansion(use_response_expansion);
  }
  return;
}

/*****************************************************************************/
void SMapper::Reset()
/*****************************************************************************/
{
  mapper_->Reset();
  return;
}

} // end namespace
