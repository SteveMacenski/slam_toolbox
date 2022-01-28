/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#ifndef SOLVERS__CERES_SOLVER_HPP_
#define SOLVERS__CERES_SOLVER_HPP_

#include <math.h>
#include <ceres/local_parameterization.h>
#include <ceres/ceres.h>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cmath>
#include "karto_sdk/Mapper.h"
#include "solvers/ceres_utils.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "slam_toolbox/toolbox_types.hpp"

namespace solver_plugins
{

using namespace ::toolbox_types;  // NOLINT

class CeresSolver : public karto::ScanSolver
{
public:
  CeresSolver();
  virtual ~CeresSolver();

public:
  // Get corrected poses after optimization
  virtual const karto::ScanSolver::IdPoseVector & GetCorrections() const;

  virtual void Compute();  // Solve
  virtual void Clear();  // Resets the corrections
  virtual void Reset();  // Resets the solver plugin clean
  virtual void Configure(rclcpp::Node::SharedPtr node);

  // Adds a node to the solver
  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex);
  // Adds a constraint to the solver
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge);
  // Get graph stored
  virtual std::unordered_map<int, Eigen::Vector3d> * getGraph();
  // Removes a node from the solver correction table
  virtual void RemoveNode(kt_int32s id);
  // Removes constraints from the optimization problem
  virtual void RemoveConstraint(kt_int32s sourceId, kt_int32s targetId);

  // change a node's pose
  virtual void ModifyNode(const int & unique_id, Eigen::Vector3d pose);
  // get a node's current pose yaw
  virtual void GetNodeOrientation(const int & unique_id, double & pose);

private:
  // karto
  karto::ScanSolver::IdPoseVector corrections_;

  // ceres
  ceres::Solver::Options options_;
  ceres::Problem::Options options_problem_;
  ceres::LossFunction * loss_function_;
  ceres::Problem * problem_;
  ceres::LocalParameterization * angle_local_parameterization_;
  bool was_constant_set_, debug_logging_;

  // graph
  std::unordered_map<int, Eigen::Vector3d> * nodes_;
  std::unordered_map<size_t, ceres::ResidualBlockId> * blocks_;
  std::unordered_map<int, Eigen::Vector3d>::iterator first_node_;
  boost::mutex nodes_mutex_;

  // ros
  rclcpp::Node::SharedPtr node_;
};

}  // namespace solver_plugins

#endif  // SOLVERS__CERES_SOLVER_HPP_
