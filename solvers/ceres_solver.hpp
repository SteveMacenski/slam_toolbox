/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#ifndef KARTO_CERESSOLVER_H
#define KARTO_CERESSOLVER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <vector>
#include <unordered_map>
#include <utility>

#include <karto_sdk/Mapper.h>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>
#include <math.h>

#include "../include/slam_toolbox/toolbox_types.hpp"
#include "ceres_utils.h"

// hash function for unordered_map of Name
namespace std {
template <>
struct hash<karto::Name>
{
  std::size_t operator()(const karto::Name& k) const
  {
    return std::hash<std::string>()(k.ToString());
  }
};

}

namespace solver_plugins
{

using namespace ::toolbox_types;

class CeresSolver : public karto::ScanSolver
{
public:
  CeresSolver();
  virtual ~CeresSolver();

public:
  virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const; //Get corrected poses after optimization
  virtual void Compute(); //Solve
  virtual void Clear(); //Resets the corrections
  virtual void Reset(); //Resets the solver plugin clean

  virtual void AddNode(karto::Name name, karto::Vertex<karto::LocalizedRangeScan>* pVertex); //Adds a node to the solver
  virtual void AddConstraint(karto::Name, karto::Edge<karto::LocalizedRangeScan>* pEdge); //Adds a constraint to the solver
  virtual std::unordered_map<int, Eigen::Vector3d>* getGraph(); //Get graph stored
  virtual void RemoveNode(kt_int32s id); //Removes a node from the solver correction table
  virtual void RemoveConstraint(karto::Name, kt_int32s sourceId, kt_int32s targetId); // Removes constraints from the optimization problem

  virtual void ModifyNode(const int& unique_id, Eigen::Vector3d pose); // change a node's pose
  virtual void GetNodeOrientation(const int& unique_id, double& pose); // get a node's current pose yaw

private:
  // karto
  karto::ScanSolver::IdPoseVector corrections_;

  // ceres
  std::unordered_map<karto::Name, ceres::Problem*> problems_;
  std::unordered_map<karto::Name, std::unordered_map<int, Eigen::Vector3d>::iterator> first_nodes_;
  ceres::Solver::Options options_;
  ceres::Problem::Options options_problem_;
  ceres::LossFunction* loss_function_;
  ceres::LocalParameterization* angle_local_parameterization_;
  bool debug_logging_;

  // graph
  std::unordered_map<int, Eigen::Vector3d>* nodes_;
  std::unordered_map<karto::Name, std::unordered_map<size_t, ceres::ResidualBlockId>* > blocks_;
  boost::mutex nodes_mutex_;
};

}

#endif
