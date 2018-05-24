/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#include "ceres_solver.hpp"
#include <open_karto/Karto.h>

#include "ros/console.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(solver_plugins::CeresSolver, karto::ScanSolver)

namespace solver_plugins
{

/*****************************************************************************/
CeresSolver::CeresSolver() : nodes_(new std::unordered_map<int, Eigen::Vector3d>()),
                             problem_(new ceres::Problem())
/*****************************************************************************/
{
  // set params
  was_constant_set_ = false;

  // formulate problem
  angle_local_parameterization_ = AngleLocalParameterization::Create();
  loss_function_ = NULL; //HuberLoss, SoftLOneLoss, CauchyLoss, CauchyLoss 
  first_node_ = nodes_->end();

  options_.max_num_iterations = 100;
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // DENSE_SCHUR, SPARSE_NORMAL_CHOLESKY, DENSE_NORMAL_CHOLESKY, DENSE_QR, SPARSE_SCHUR, ITERATIVE_SCHUR, CGNR
  options_.num_threads = 5;

  /*
  options_minimizer_type = LEVENBERG_MARQUARDT;
  options_.tau = 1e-4;
  options_.min_mu = 1e-20;
  options_.min_relative_decrease = 1e-3;
  options_.function_tolerance = 1e-6;
  options_.gradient_tolerance = 1e-10;
  options_.parameter_tolerance = 1e-8;
  options_.preconditioner_type = JACOBI;
  options_.num_linear_solver_threads = 1;
  options_.num_eliminate_blocks = 0;
  options_.ordering_type = NATURAL;
  options_.linear_solver_max_num_iterations = 500;
  options_.eta = 1e-1;
  options_.jacobi_scaling = true;
  options_.logging_type = PER_MINIMIZER_ITERATION;
  options_.minimizer_progress_to_stdout = false;
  options_.check_gradients = false;
  options_.gradient_check_relative_precision = 1e-8;
  options_.numeric_derivative_relative_step_size = 1e-6;
  */
}

/*****************************************************************************/
CeresSolver::~CeresSolver()
/*****************************************************************************/
{
  if ( loss_function_ != NULL)
  {
    delete loss_function_;
  }
  if (nodes_ != NULL)
  {
    delete nodes_;
  }
  if (problem_ != NULL)
  {
    delete problem_;  
  }
  if (angle_local_parameterization_ != NULL)
  {
    delete angle_local_parameterization_;
  }
}

/*****************************************************************************/
void CeresSolver::Compute()
/*****************************************************************************/
{
  if (nodes_->size() == 0)
  {
    ROS_ERROR("Ceres was called when there are no nodes. This shouldn't happen.");
    return;
  }

  // populate contraint for static initial pose
  if (!was_constant_set_ && first_node_ != nodes_->end())
  {
    problem_->SetParameterBlockConstant(&first_node_->second(0));
    problem_->SetParameterBlockConstant(&first_node_->second(1));
    problem_->SetParameterBlockConstant(&first_node_->second(2));
    was_constant_set_ = !was_constant_set_;
  }

  const ros::Time start_time = ros::Time::now();
  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);
  ROS_INFO("Loop Closure Solve time: %f seconds", (ros::Time::now() - start_time).toSec());

  if (!summary.IsSolutionUsable())
  {
    ROS_WARN("Ceres could not find a usable solution to optimize.");
    return;
  }

  // store corrected poses
  if (!corrections_.empty())
  {
    corrections_.clear();
  }
  corrections_.reserve(nodes_->size());
  karto::Pose2 pose;
  std::unordered_map<int, Eigen::Vector3d>::const_iterator iter = nodes_->begin();
  for ( iter; iter != nodes_->end(); ++iter )
  {
    pose.SetX(iter->second(0));
    pose.SetY(iter->second(1));
    pose.SetHeading(iter->second(2));
    corrections_.push_back(std::make_pair(iter->first, pose));
  }
  return;
}

/*****************************************************************************/
const karto::ScanSolver::IdPoseVector& CeresSolver::GetCorrections() const
/*****************************************************************************/
{
  return corrections_;
}

/*****************************************************************************/
void CeresSolver::Clear()
/*****************************************************************************/
{
  corrections_.clear();
}

/*****************************************************************************/
void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
/*****************************************************************************/
{
  // store nodes
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d pose2d(pose.GetX(), pose.GetY(), pose.GetHeading());

  const int id = pVertex->GetObject()->GetUniqueId();

  boost::mutex::scoped_lock lock(nodes_mutex_);
  nodes_->insert(std::pair<int,Eigen::Vector3d>(id,pose2d));

  if (nodes_->size() == 1)
  {
    first_node_ = nodes_->find(id);
  }
}

/*****************************************************************************/
void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
/*****************************************************************************/
{
  // get IDs in graph for this edge
  boost::mutex::scoped_lock lock(nodes_mutex_);
  const int node1 = pEdge->GetSource()->GetObject()->GetUniqueId();
  std::unordered_map<int, Eigen::Vector3d>::iterator node1it = nodes_->find(node1);
  const int node2 = pEdge->GetTarget()->GetObject()->GetUniqueId();
  std::unordered_map<int, Eigen::Vector3d>::iterator node2it = nodes_->find(node2);

  if (node1it ==  nodes_->end() || node2it == nodes_->end() || node1it == node2it)
  {
    ROS_WARN("Failed to add constraint, could not find nodes.");
    return;
  }

  // extract transformation
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  Eigen::Vector3d pose2d(diff.GetX(), diff.GetY(), diff.GetHeading());

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix3d sqrt_information;
  sqrt_information(0,0) = precisionMatrix(0,0);
  sqrt_information(0,1) = sqrt_information(1,0) = precisionMatrix(0,1);
  sqrt_information(0,2) = sqrt_information(2,0) = precisionMatrix(0,2);
  sqrt_information(1,1) = precisionMatrix(1,1);
  sqrt_information(1,2) = sqrt_information(2,1) = precisionMatrix(1,2);
  sqrt_information(2,2) = precisionMatrix(2,2);

  // populate residual and parameterization for heading normalization
  ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create( pose2d(0), 
                                            pose2d(1), pose2d(2), sqrt_information);
  problem_->AddResidualBlock( cost_function, loss_function_, 
                     &node1it->second(0), &node1it->second(1), &node1it->second(2),
                     &node2it->second(0), &node2it->second(1), &node2it->second(2));
  problem_->SetParameterization(&node1it->second(2), angle_local_parameterization_);
  problem_->SetParameterization(&node2it->second(2), angle_local_parameterization_);
  return;
}

/*****************************************************************************/
void CeresSolver::getGraph(std::vector<Eigen::Vector2d> &g)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  g.reserve(nodes_->size());
  std::unordered_map<int,Eigen::Vector3d>::const_iterator it = nodes_->begin();
  for (it; it!=nodes_->end(); ++it)
  {
    g.push_back(Eigen::Vector2d(it->second(0), it->second(1)));
  }
  return;
}

} // end namespace
