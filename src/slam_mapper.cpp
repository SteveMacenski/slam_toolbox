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
    resolution);
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
void SMapper::configure(
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface)
/*****************************************************************************/
{
  bool use_scan_matching = true;
  if (!parameters_interface->has_parameter("use_scan_matching")) {
    parameters_interface->declare_parameter("use_scan_matching",
                                            rclcpp::ParameterValue(use_scan_matching));
  }
  use_scan_matching = parameters_interface->get_parameter("use_scan_matching").as_bool();
  mapper_->setParamUseScanMatching(use_scan_matching);

  bool use_scan_barycenter = true;
  if (!parameters_interface->has_parameter("use_scan_barycenter")) {
    parameters_interface->declare_parameter("use_scan_barycenter",
                                            rclcpp::ParameterValue(use_scan_barycenter));
  }
  use_scan_barycenter = parameters_interface->get_parameter("use_scan_barycenter").as_bool();
  mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_travel_distance = 0.5;
  if (!parameters_interface->has_parameter("minimum_travel_distance")) {
    parameters_interface->declare_parameter("minimum_travel_distance",
                                            rclcpp::ParameterValue(minimum_travel_distance));
  }
  minimum_travel_distance = parameters_interface->get_parameter("minimum_travel_distance").as_double();
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  double minimum_travel_heading = 0.5;
  if (!parameters_interface->has_parameter("minimum_travel_heading")) {
    parameters_interface->declare_parameter("minimum_travel_heading",
                                            rclcpp::ParameterValue(minimum_travel_heading));
  }
  minimum_travel_heading = parameters_interface->get_parameter("minimum_travel_heading").as_double();
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size = 10;
  if (!parameters_interface->has_parameter("scan_buffer_size")) {
    parameters_interface->declare_parameter("scan_buffer_size",
                                            rclcpp::ParameterValue(scan_buffer_size));
  }
  scan_buffer_size = parameters_interface->get_parameter("scan_buffer_size").as_int();
  if (scan_buffer_size <= 0) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set scan_buffer_size to be a value smaller than zero,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_size = 10;
  }
  mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance = 10;
  if (!parameters_interface->has_parameter("scan_buffer_maximum_scan_distance")) {
    parameters_interface->declare_parameter("scan_buffer_maximum_scan_distance",
                                            rclcpp::ParameterValue(scan_buffer_maximum_scan_distance));
  }
  scan_buffer_maximum_scan_distance = parameters_interface->get_parameter("scan_buffer_maximum_scan_distance").as_double();
  if (math::Square(scan_buffer_maximum_scan_distance) <= 1e-06) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set scan_buffer_maximum_scan_distance to be a value whose square is smaller than 1e-06,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_maximum_scan_distance = 10;
  }
  mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine = 0.1;
  if (!parameters_interface->has_parameter("link_match_minimum_response_fine")) {
    parameters_interface->declare_parameter("link_match_minimum_response_fine",
                                            rclcpp::ParameterValue(link_match_minimum_response_fine));
  }
  link_match_minimum_response_fine = parameters_interface->get_parameter("link_match_minimum_response_fine").as_double();
  mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance = 1.5;
  if (!parameters_interface->has_parameter("link_scan_maximum_distance")) {
    parameters_interface->declare_parameter("link_scan_maximum_distance",
                                            rclcpp::ParameterValue(link_scan_maximum_distance));
  }
  link_scan_maximum_distance = parameters_interface->get_parameter("link_scan_maximum_distance").as_double();
  mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance = 3.0;
  if (!parameters_interface->has_parameter("loop_search_maximum_distance")) {
    parameters_interface->declare_parameter("loop_search_maximum_distance",
                                            rclcpp::ParameterValue(loop_search_maximum_distance));
  }
  loop_search_maximum_distance = parameters_interface->get_parameter("loop_search_maximum_distance").as_double();
  mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing = true;
  if (!parameters_interface->has_parameter("do_loop_closing")) {
    parameters_interface->declare_parameter("do_loop_closing",
                                            rclcpp::ParameterValue(do_loop_closing));
  }
  do_loop_closing = parameters_interface->get_parameter("do_loop_closing").as_bool();
  mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size = 10;
  if (!parameters_interface->has_parameter("loop_match_minimum_chain_size")) {
    parameters_interface->declare_parameter("loop_match_minimum_chain_size",
                                            rclcpp::ParameterValue(loop_match_minimum_chain_size));
  }
  loop_match_minimum_chain_size = parameters_interface->get_parameter("loop_match_minimum_chain_size").as_int();
  mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse = 3.0;
  if (!parameters_interface->has_parameter("loop_match_maximum_variance_coarse")) {
    parameters_interface->declare_parameter("loop_match_maximum_variance_coarse",
                                            rclcpp::ParameterValue(loop_match_maximum_variance_coarse));
  }
  loop_match_maximum_variance_coarse = parameters_interface->get_parameter("loop_match_maximum_variance_coarse").as_double();
  mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse = 0.35;
  if (!parameters_interface->has_parameter("loop_match_minimum_response_coarse")) {
    parameters_interface->declare_parameter("loop_match_minimum_response_coarse",
                                            rclcpp::ParameterValue(loop_match_minimum_response_coarse));
  }
  loop_match_minimum_response_coarse = parameters_interface->get_parameter("loop_match_minimum_response_coarse").as_double();
  mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine = 0.45;
  if (!parameters_interface->has_parameter("loop_match_minimum_response_fine")) {
    parameters_interface->declare_parameter("loop_match_minimum_response_fine",
                                            rclcpp::ParameterValue(loop_match_minimum_response_fine));
  }
  loop_match_minimum_response_fine = parameters_interface->get_parameter("loop_match_minimum_response_fine").as_double();
  mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters
  double correlation_search_space_dimension = 0.5;
  if (!parameters_interface->has_parameter("correlation_search_space_dimension")) {
    parameters_interface->declare_parameter("correlation_search_space_dimension",
                                            rclcpp::ParameterValue(correlation_search_space_dimension));
  }
  correlation_search_space_dimension = parameters_interface->get_parameter("correlation_search_space_dimension").as_double();
  if (correlation_search_space_dimension <= 0) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set correlation_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 0.5.");
    correlation_search_space_dimension = 0.5;
  }
  mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution = 0.01;
  if (!parameters_interface->has_parameter("correlation_search_space_resolution")) {
    parameters_interface->declare_parameter("correlation_search_space_resolution",
                                            rclcpp::ParameterValue(correlation_search_space_resolution));
  }
  correlation_search_space_resolution = parameters_interface->get_parameter("correlation_search_space_resolution").as_double();
  if (correlation_search_space_resolution <= 0) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set correlation_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.01.");
    correlation_search_space_resolution = 0.01;
  }
  mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation = 0.1;
  if (!parameters_interface->has_parameter("correlation_search_space_smear_deviation")) {
    parameters_interface->declare_parameter("correlation_search_space_smear_deviation",
                                            rclcpp::ParameterValue(correlation_search_space_smear_deviation));
  }
  correlation_search_space_smear_deviation = parameters_interface->get_parameter("correlation_search_space_smear_deviation").as_double();
  if (correlation_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set correlation_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.1.");
    correlation_search_space_smear_deviation = 0.1;
  }
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension = 8.0;
  if (!parameters_interface->has_parameter("loop_search_space_dimension")) {
    parameters_interface->declare_parameter("loop_search_space_dimension",
                                            rclcpp::ParameterValue(loop_search_space_dimension));
  }
  loop_search_space_dimension = parameters_interface->get_parameter("loop_search_space_dimension").as_double();
  if (loop_search_space_dimension <= 0) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set loop_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 8.0.");
    loop_search_space_dimension = 8.0;
  }
  mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution = 0.05;
  if (!parameters_interface->has_parameter("loop_search_space_resolution")) {
    parameters_interface->declare_parameter("loop_search_space_resolution",
                                            rclcpp::ParameterValue(loop_search_space_resolution));
  }
  loop_search_space_resolution = parameters_interface->get_parameter("loop_search_space_resolution").as_double();
  if (loop_search_space_resolution <= 0) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set loop_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.05.");
    loop_search_space_resolution = 0.05;
  }
  mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation = 0.03;
  if (!parameters_interface->has_parameter("loop_search_space_smear_deviation")) {
    parameters_interface->declare_parameter("loop_search_space_smear_deviation",
                                            rclcpp::ParameterValue(loop_search_space_smear_deviation));
  }
  loop_search_space_smear_deviation = parameters_interface->get_parameter("loop_search_space_smear_deviation").as_double();
  if (loop_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(logging_interface->get_logger(),
      "You've set loop_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.03.");
    loop_search_space_smear_deviation = 0.03;
  }
  mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters
  double distance_variance_penalty = 0.5;
  if (!parameters_interface->has_parameter("distance_variance_penalty")) {
    parameters_interface->declare_parameter("distance_variance_penalty",
                                            rclcpp::ParameterValue(distance_variance_penalty));
  }
  distance_variance_penalty = parameters_interface->get_parameter("distance_variance_penalty").as_double();
  mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty = 1.0;
  if (!parameters_interface->has_parameter("angle_variance_penalty")) {
    parameters_interface->declare_parameter("angle_variance_penalty",
                                            rclcpp::ParameterValue(angle_variance_penalty));
  }
  angle_variance_penalty = parameters_interface->get_parameter("angle_variance_penalty").as_double();
  mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset = 0.00349;
  if (!parameters_interface->has_parameter("fine_search_angle_offset")) {
    parameters_interface->declare_parameter("fine_search_angle_offset",
                                            rclcpp::ParameterValue(fine_search_angle_offset));
  }
  fine_search_angle_offset = parameters_interface->get_parameter("fine_search_angle_offset").as_double();
  mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset = 0.349;
  if (!parameters_interface->has_parameter("coarse_search_angle_offset")) {
    parameters_interface->declare_parameter("coarse_search_angle_offset",
                                            rclcpp::ParameterValue(coarse_search_angle_offset));
  }
  coarse_search_angle_offset = parameters_interface->get_parameter("coarse_search_angle_offset").as_double();
  mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution = 0.0349;
  if (!parameters_interface->has_parameter("coarse_angle_resolution")) {
    parameters_interface->declare_parameter("coarse_angle_resolution",
                                            rclcpp::ParameterValue(coarse_angle_resolution));
  }
  coarse_angle_resolution = parameters_interface->get_parameter("coarse_angle_resolution").as_double();
  mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty = 0.9;
  if (!parameters_interface->has_parameter("minimum_angle_penalty")) {
    parameters_interface->declare_parameter("minimum_angle_penalty",
                                            rclcpp::ParameterValue(minimum_angle_penalty));
  }
  minimum_angle_penalty = parameters_interface->get_parameter("minimum_angle_penalty").as_double();
  mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty = 0.05;
  if (!parameters_interface->has_parameter("minimum_distance_penalty")) {
    parameters_interface->declare_parameter("minimum_distance_penalty",
                                            rclcpp::ParameterValue(minimum_distance_penalty));
  }
  minimum_distance_penalty = parameters_interface->get_parameter("minimum_distance_penalty").as_double();
  mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion = true;
  if (!parameters_interface->has_parameter("use_response_expansion")) {
    parameters_interface->declare_parameter("use_response_expansion",
                                            rclcpp::ParameterValue(use_response_expansion));
  }
  use_response_expansion = parameters_interface->get_parameter("use_response_expansion").as_bool();
  mapper_->setParamUseResponseExpansion(use_response_expansion);
}

/*****************************************************************************/
void SMapper::Reset()
/*****************************************************************************/
{
  mapper_->Reset();
}

}  // namespace mapper_utils
