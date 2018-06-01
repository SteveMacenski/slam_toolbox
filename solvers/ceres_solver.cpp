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
                             problem_(new ceres::Problem()), was_constant_set_(false)
/*****************************************************************************/
{
  ros::NodeHandle nh;
  _times_server = nh.advertiseService("/get_times", &CeresSolver::get_times, this);

  corrections_.clear();

  // formulate problem
  angle_local_parameterization_ = AngleLocalParameterization::Create();
  loss_function_ = NULL; //HuberLoss, CauchyLoss
  first_node_ = nodes_->end();

  options_.linear_solver_type = ceres::SPARSE_SCHUR; // SPARSE_NORMAL_CHOLESKY, SPARSE_SCHUR
  options_.preconditioner_type = ceres::IDENTITY; //JACOBI; IDENTITY, **SCHUR_JACOBI

  if (options_.preconditioner_type == ceres::CLUSTER_JACOBI || options_.preconditioner_type == ceres::CLUSTER_TRIDIAGONAL)
  {
    options_.visibility_clustering_type = ceres::CANONICAL_VIEWS; //CANONICAL_VIEWS ***SINGLE_LINKAGE 
  }

  options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; //LEVENBERG_MARQUARDT, DOGLEG 

  if(options_.trust_region_strategy_type == ceres::DOGLEG)
  {
    options_.dogleg_type = ceres::TRADITIONAL_DOGLEG; //TRADITIONAL_DOGLEG, SUBSPACE_DOGLEG
  }

  // could be that residual is too small so autodifferentiation is too slow
  // could be that we have a pretty "solved" problem we're refining, bound more




  options_.jacobi_scaling = true; //true

  options_.function_tolerance = 1e-2; //1e-6    //e-10 F
  options_.gradient_tolerance = 1e-6; //1e-10
  options_.parameter_tolerance = 1e-4; //1e-8
  options_.eta = 1e-1; //1e-1   Ge-2

  options_.use_nonmonotonic_steps = true; // false
  options_.min_relative_decrease = 1e-3; //1e-3

  options_.initial_trust_region_radius = 1e4; //1e4
  options_.max_trust_region_radius = 1e8; //1e16
  options_.min_trust_region_radius = 1e-16; //1e-32

  options_.min_lm_diagonal = 1e-6; //1e-6
  options_.max_lm_diagonal = 1e32; //1e32

  // immutable.
  options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options_.minimizer_type = ceres::TRUST_REGION;
  options_.max_num_iterations = 50;
  options_.max_num_consecutive_invalid_steps = 3;
  options_.max_consecutive_nonmonotonic_steps = 3;
  options_.num_threads = 50;
  if(options_.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY)
  {
    options_.dynamic_sparsity = true; // ~10-20% speed up as graph grows with CHOL
  }
  else if(options_.linear_solver_type == ceres::ITERATIVE_SCHUR)
  {
    options_.use_explicit_schur_complement = false; // can help significantly for small to medium sized problems
  }
}

bool CeresSolver::get_times(std_srvs::Empty::Request& e, std_srvs::Empty::Response& e2)
{
  std::string s;
  for (int i=0; i!=_times.size();i++)
  {
    s += " " + std::to_string(_times[i]);
  }
  ROS_INFO("%s", s.c_str());
  return true;
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
    ROS_ERROR("CeresSolver: Ceres was called when there are no nodes."
              " This shouldn't happen.");
    return;
  }

  // populate contraint for static initial pose
  if (!was_constant_set_ && first_node_ != nodes_->end())
  {
    ROS_INFO("CeresSolver: Setting first node as a constant pose.");
    problem_->SetParameterBlockConstant(&first_node_->second(0));
    problem_->SetParameterBlockConstant(&first_node_->second(1));
    problem_->SetParameterBlockConstant(&first_node_->second(2));
    was_constant_set_ = !was_constant_set_;
  }

  const ros::Time start_time = ros::Time::now();
  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);
  std::cout << summary.FullReport() << '\n';
  _times.push_back((ros::Time::now() - start_time).toSec());
  ROS_INFO("Loop Closure Solve time: %f seconds", _times.back());

  if (!summary.IsSolutionUsable())
  {
    ROS_WARN("CeresSolver: "
                          "Ceres could not find a usable solution to optimize.");
    return;
  }

  // store corrected poses
  if (!corrections_.empty())
  {
    corrections_.clear();
  }
  corrections_.reserve(nodes_->size());
  karto::Pose2 pose;
  const_graph_iterator iter = nodes_->begin();
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
  graph_iterator node1it = nodes_->find(node1);
  const int node2 = pEdge->GetTarget()->GetObject()->GetUniqueId();
  graph_iterator node2it = nodes_->find(node2);

  if (node1it ==  nodes_->end() || node2it == nodes_->end() || node1it == node2it)
  {
    ROS_WARN("CeresSolver: Failed to add constraint, could not find nodes.");
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
void CeresSolver::ModifyNode(const int& unique_id, Eigen::Vector3d& pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  graph_iterator it = nodes_->find(unique_id);
  if (it != nodes_->end())
  {
    it->second = pose;
  }
}

/*****************************************************************************/
void CeresSolver::getGraph(std::vector<Eigen::Vector2d> &g)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  g.resize(nodes_->size());
  const_graph_iterator it = nodes_->begin();
  for (it; it!=nodes_->end(); ++it)
  {
    g[it->first] = Eigen::Vector2d(it->second(0), it->second(1));
  }
  return;
}

} // end namespace
