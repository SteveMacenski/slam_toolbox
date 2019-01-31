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

#include "ceres_utils.h"

namespace solver_plugins
{

typedef std::vector<karto::Matrix3> CovarianceVector;
typedef std::unordered_map<int, Eigen::Vector3d>::iterator graph_iterator;
typedef std::unordered_map<int, Eigen::Vector3d>::const_iterator const_graph_iterator;

class CeresSolver : public karto::ScanSolver
{
public:
  CeresSolver();
  virtual ~CeresSolver();

public:
  virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const; //Get corrected poses after optimization
  virtual void Compute(); //Solve
  virtual void Clear(); //Resets the corrections

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex); //Adds a node to the solver
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge); //Adds a constraint to the solver
  virtual void getGraph(std::vector<Eigen::Vector2d> &g); //Get graph stored

  virtual void ModifyNode(const int& unique_id, Eigen::Vector3d pose); // change a node's pose
  virtual void GetNodeOrientation(const int& unique_id, double& pose); // get a node's current pose yaw

private:
  // karto
  karto::ScanSolver::IdPoseVector corrections_;

  // ceres
  ceres::Solver::Options options_;
  ceres::LossFunction* loss_function_;
  ceres::Problem* problem_;
  ceres::LocalParameterization* angle_local_parameterization_;
  bool was_constant_set_;

  // graph
  std::unordered_map<int, Eigen::Vector3d>* nodes_;
  std::unordered_map<int, Eigen::Vector3d>::iterator first_node_;
  boost::mutex nodes_mutex_;
};

}

#endif
