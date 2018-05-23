/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#ifndef KARTO_CERESSOLVER_H
#define KARTO_CERESSOLVER_H

#include <vector>
#include <unordered_map>
#include <utility>

#include <open_karto/Mapper.h>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>

#include "ceres_utils.h"

namespace solver_plugins
{

typedef std::vector<karto::Matrix3> CovarianceVector;

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
  Eigen::Vector3d first_node_;
};

}

#endif
