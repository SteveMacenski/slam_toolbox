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

#include <memory>
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
karto::Mapper * SMapper::getMapper()
/*****************************************************************************/
{
  return mapper_.get();
}

/*****************************************************************************/
void SMapper::setMapper(karto::Mapper * mapper)
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
karto::OccupancyGrid * SMapper::getOccupancyGrid(const double & resolution)
/*****************************************************************************/
{
  karto::OccupancyGrid * occ_grid = nullptr;
  return karto::OccupancyGrid::CreateFromScans(
    mapper_->GetAllProcessedScans(),
    resolution, (kt_int32u)mapper_->getParamMinPassThrough(), (kt_double)mapper_->getParamOccupancyThreshold());
}

/*****************************************************************************/
tf2::Transform SMapper::toTfPose(const karto::Pose2 & pose) const
/*****************************************************************************/
{
  tf2::Transform new_pose;
  new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  tf2::Quaternion q;
  q.setRPY(0., 0., pose.GetHeading());
  new_pose.setRotation(q);
  return new_pose;
}

/*****************************************************************************/
karto::Pose2 SMapper::toKartoPose(const tf2::Transform & pose) const
/*****************************************************************************/
{
  karto::Pose2 transformed_pose;
  transformed_pose.SetX(pose.getOrigin().x());
  transformed_pose.SetY(pose.getOrigin().y());
  transformed_pose.SetHeading(tf2::getYaw(pose.getRotation()));
  return transformed_pose;
}

/*****************************************************************************/
template<class NodeT>
void SMapper::configure(const NodeT & node)
/*****************************************************************************/
{
  bool use_scan_matching = true;
  if (!node->has_parameter("use_scan_matching")) {
    node->declare_parameter("use_scan_matching", use_scan_matching);
  }
  node->get_parameter("use_scan_matching", use_scan_matching);
  mapper_->setParamUseScanMatching(use_scan_matching);

  bool use_scan_barycenter = true;
  if (!node->has_parameter("use_scan_barycenter")) {
    node->declare_parameter("use_scan_barycenter", use_scan_barycenter);
  }
  node->get_parameter("use_scan_barycenter", use_scan_barycenter);
  mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_travel_distance = 0.5;
  if (!node->has_parameter("minimum_travel_distance")) {
    node->declare_parameter("minimum_travel_distance", minimum_travel_distance);
  }
  node->get_parameter("minimum_travel_distance", minimum_travel_distance);
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  double minimum_travel_heading = 0.5;
  if (!node->has_parameter("minimum_travel_heading")) {
    node->declare_parameter("minimum_travel_heading", minimum_travel_heading);
  }
  node->get_parameter("minimum_travel_heading", minimum_travel_heading);
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size = 10;
  if (!node->has_parameter("scan_buffer_size")) {
    node->declare_parameter("scan_buffer_size", scan_buffer_size);
  }
  node->get_parameter("scan_buffer_size", scan_buffer_size);
  if (scan_buffer_size <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set scan_buffer_size to be a value smaller than zero,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_size = 10;
  }
  mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance = 10;
  if (!node->has_parameter("scan_buffer_maximum_scan_distance")) {
    node->declare_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance);
  }
  node->get_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance);
  if (math::Square(scan_buffer_maximum_scan_distance) <= 1e-06) {
    RCLCPP_WARN(node->get_logger(),
      "You've set scan_buffer_maximum_scan_distance to be a value whose square is smaller than 1e-06,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_maximum_scan_distance = 10;
  }
  mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine = 0.1;
  if (!node->has_parameter("link_match_minimum_response_fine")) {
    node->declare_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine);
  }
  node->get_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine);
  mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance = 1.5;
  if (!node->has_parameter("link_scan_maximum_distance")) {
    node->declare_parameter("link_scan_maximum_distance", link_scan_maximum_distance);
  }
  node->get_parameter("link_scan_maximum_distance", link_scan_maximum_distance);
  mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance = 3.0;
  if (!node->has_parameter("loop_search_maximum_distance")) {
    node->declare_parameter("loop_search_maximum_distance", loop_search_maximum_distance);
  }
  node->get_parameter("loop_search_maximum_distance", loop_search_maximum_distance);
  mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing = true;
  if (!node->has_parameter("do_loop_closing")) {
    node->declare_parameter("do_loop_closing", do_loop_closing);
  }
  node->get_parameter("do_loop_closing", do_loop_closing);
  mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size = 10;
  if (!node->has_parameter("loop_match_minimum_chain_size")) {
    node->declare_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size);
  }
  node->get_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size);
  mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse = 3.0;
  if (!node->has_parameter("loop_match_maximum_variance_coarse")) {
    node->declare_parameter(
      "loop_match_maximum_variance_coarse",
      loop_match_maximum_variance_coarse);
  }
  node->get_parameter("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse);
  mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse = 0.35;
  if (!node->has_parameter("loop_match_minimum_response_coarse")) {
    node->declare_parameter(
      "loop_match_minimum_response_coarse",
      loop_match_minimum_response_coarse);
  }
  node->get_parameter("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse);
  mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine = 0.45;
  if (!node->has_parameter("loop_match_minimum_response_fine")) {
    node->declare_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine);
  }
  node->get_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine);
  mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters
  double correlation_search_space_dimension = 0.5;
  if (!node->has_parameter("correlation_search_space_dimension")) {
    node->declare_parameter(
      "correlation_search_space_dimension",
      correlation_search_space_dimension);
  }
  node->get_parameter("correlation_search_space_dimension", correlation_search_space_dimension);
  if (correlation_search_space_dimension <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 0.5.");
    correlation_search_space_dimension = 0.5;
  }
  mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution = 0.01;
  if (!node->has_parameter("correlation_search_space_resolution")) {
    node->declare_parameter(
      "correlation_search_space_resolution",
      correlation_search_space_resolution);
  }
  node->get_parameter("correlation_search_space_resolution", correlation_search_space_resolution);
  if (correlation_search_space_resolution <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.01.");
    correlation_search_space_resolution = 0.01;
  }
  mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation = 0.1;
  if (!node->has_parameter("correlation_search_space_smear_deviation")) {
    node->declare_parameter(
      "correlation_search_space_smear_deviation",
      correlation_search_space_smear_deviation);
  }
  node->get_parameter(
    "correlation_search_space_smear_deviation",
    correlation_search_space_smear_deviation);
  if (correlation_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.1.");
    correlation_search_space_smear_deviation = 0.1;
  }
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension = 8.0;
  if (!node->has_parameter("loop_search_space_dimension")) {
    node->declare_parameter("loop_search_space_dimension", loop_search_space_dimension);
  }
  node->get_parameter("loop_search_space_dimension", loop_search_space_dimension);
  if (loop_search_space_dimension <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 8.0.");
    loop_search_space_dimension = 8.0;
  }
  mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution = 0.05;
  if (!node->has_parameter("loop_search_space_resolution")) {
    node->declare_parameter("loop_search_space_resolution", loop_search_space_resolution);
  }
  node->get_parameter("loop_search_space_resolution", loop_search_space_resolution);
  if (loop_search_space_resolution <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.05.");
    loop_search_space_resolution = 0.05;
  }
  mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation = 0.03;
  if (!node->has_parameter("loop_search_space_smear_deviation")) {
    node->declare_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation);
  }
  node->get_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation);
  if (loop_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.03.");
    loop_search_space_smear_deviation = 0.03;
  }
  mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters
  double distance_variance_penalty = 0.5;
  if (!node->has_parameter("distance_variance_penalty")) {
    node->declare_parameter("distance_variance_penalty", distance_variance_penalty);
  }
  node->get_parameter("distance_variance_penalty", distance_variance_penalty);
  mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty = 1.0;
  if (!node->has_parameter("angle_variance_penalty")) {
    node->declare_parameter("angle_variance_penalty", angle_variance_penalty);
  }
  node->get_parameter("angle_variance_penalty", angle_variance_penalty);
  mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset = 0.00349;
  if (!node->has_parameter("fine_search_angle_offset")) {
    node->declare_parameter("fine_search_angle_offset", fine_search_angle_offset);
  }
  node->get_parameter("fine_search_angle_offset", fine_search_angle_offset);
  mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset = 0.349;
  if (!node->has_parameter("coarse_search_angle_offset")) {
    node->declare_parameter("coarse_search_angle_offset", coarse_search_angle_offset);
  }
  node->get_parameter("coarse_search_angle_offset", coarse_search_angle_offset);
  mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution = 0.0349;
  if (!node->has_parameter("coarse_angle_resolution")) {
    node->declare_parameter("coarse_angle_resolution", coarse_angle_resolution);
  }
  node->get_parameter("coarse_angle_resolution", coarse_angle_resolution);
  mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty = 0.9;
  if (!node->has_parameter("minimum_angle_penalty")) {
    node->declare_parameter("minimum_angle_penalty", minimum_angle_penalty);
  }
  node->get_parameter("minimum_angle_penalty", minimum_angle_penalty);
  mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty = 0.05;
  if (!node->has_parameter("minimum_distance_penalty")) {
    node->declare_parameter("minimum_distance_penalty", minimum_distance_penalty);
  }
  node->get_parameter("minimum_distance_penalty", minimum_distance_penalty);
  mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion = true;
  if (!node->has_parameter("use_response_expansion")) {
    node->declare_parameter("use_response_expansion", use_response_expansion);
  }
  node->get_parameter("use_response_expansion", use_response_expansion);
  mapper_->setParamUseResponseExpansion(use_response_expansion);


  int min_pass_through = 2;
  if (!node->has_parameter("min_pass_through")) {
    node->declare_parameter("min_pass_through", min_pass_through);
  }
  node->get_parameter("min_pass_through", min_pass_through);
  mapper_->setParamMinPassThrough(min_pass_through);

  double occupancy_threshold = 0.1;
  if (!node->has_parameter("occupancy_threshold")) {
    node->declare_parameter("occupancy_threshold", occupancy_threshold);
  }
  node->get_parameter("occupancy_threshold", occupancy_threshold);
  mapper_->setParamOccupancyThreshold(occupancy_threshold);
}

/*****************************************************************************/
void SMapper::Reset()
/*****************************************************************************/
{
  mapper_->Reset();
}

// explicit instantiation for the supported template types
template void SMapper::configure(const rclcpp::Node::SharedPtr &);
template void SMapper::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &);

}  // namespace mapper_utils
