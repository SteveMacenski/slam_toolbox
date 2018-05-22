/*
 * Copyright 2018 Simbe Robotics
 * Author: Steve Macenski
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
                             problem_(new ceres::Problem()),
                             node_vec_(new std::vector<Eigen::Vector2d>())
/*****************************************************************************/
{
  // set params
  was_constant_set_ = false;

  // formulate problem
  angle_local_parameterization_ = AngleLocalParameterization::Create();
  loss_function_ = NULL; //TODO

  options_.max_num_iterations = 100;
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; //ceres::DENSE_SCHUR
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
  if (node_vec_ != NULL)
  {
    delete node_vec_;
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
  ROS_INFO("Compute started");

  if (nodes_->size() == 0)
  {
    ROS_ERROR("Ceres was called when there are no nodes. This shouldn't happen.");
    return;
  }

  // populate contraint for static initial pose
  // if (!was_constant_set_)
  // {
  //   problem_->SetParameterBlockConstant(&first_node_(0));
  //   problem_->SetParameterBlockConstant(&first_node_(1));
  //   problem_->SetParameterBlockConstant(&first_node_(2));
  //   was_constant_set_ = !was_constant_set_;
  // }
  ROS_INFO("Solver started");
  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);
  ROS_INFO("Solver ended");

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
    //const karto::Pose2 pose(iter->second(0), iter->second(1), iter->second(2)); //TODO check efficiency of adding this way. about 100ms after 2.5 aisles
    corrections_.push_back(std::make_pair(iter->first, pose));
  }
  ROS_INFO("Ended correction updates");
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

  if (nodes_->size() == 0)
  {
    first_node_ = pose2d;
  }

  nodes_->insert(std::pair<int,Eigen::Vector3d>(pVertex->GetObject()->GetUniqueId(),pose2d));
  node_vec_->push_back(Eigen::Vector2d(pose2d(0), pose2d(1)));
}

/*****************************************************************************/
void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
/*****************************************************************************/
{
  // store edges, for now just add to graph

  // get IDs in graph for this edge
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
}

/*****************************************************************************/
void CeresSolver::getGraph(std::vector<Eigen::Vector2d> &g)
/*****************************************************************************/
{
  g = *node_vec_;
}

} // end namespace
