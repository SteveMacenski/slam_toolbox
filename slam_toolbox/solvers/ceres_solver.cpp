/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#include "ceres_solver.hpp"
#include <karto_sdk/Karto.h>

#include "ros/console.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(solver_plugins::CeresSolver, karto::ScanSolver)

namespace solver_plugins
{

/*****************************************************************************/
CeresSolver::CeresSolver() : 
  problem_(nullptr), was_constant_set_(false),
  parameter_blocks_(new ParameterBlockMap()),
  residual_blocks_(new ResidualBlockMap()),
  parameter_block_pool_(sizeof(double) * 3u),
  first_parameter_block_(nullptr)
/*****************************************************************************/
{
  ros::NodeHandle nh("~");
  std::string solver_type, preconditioner_type, dogleg_type,
    trust_strategy, loss_fn, mode;
  nh.getParam("ceres_linear_solver", solver_type);
  nh.getParam("ceres_preconditioner", preconditioner_type);
  nh.getParam("ceres_dogleg_type", dogleg_type);
  nh.getParam("ceres_trust_strategy", trust_strategy);
  nh.getParam("ceres_loss_function", loss_fn);
  nh.getParam("mode", mode);
  nh.getParam("debug_logging", debug_logging_);

  // formulate problem
  local_parameterization_ = new ceres::ProductParameterization(
      new ceres::IdentityParameterization(2),
      AngleLocalParameterization::Create());

  // choose loss function default squared loss (NULL)
  loss_function_ = NULL;
  if (loss_fn == "HuberLoss")
  {
    ROS_INFO("CeresSolver: Using HuberLoss loss function.");
    loss_function_ = new ceres::HuberLoss(0.7);
  }
  else if (loss_fn == "CauchyLoss")
  {
    ROS_INFO("CeresSolver: Using CauchyLoss loss function.");
    loss_function_ = new ceres::CauchyLoss(0.7);
  }

  // choose linear solver default CHOL
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (solver_type == "SPARSE_SCHUR")
  {
    ROS_INFO("CeresSolver: Using SPARSE_SCHUR solver.");
    options_.linear_solver_type = ceres::SPARSE_SCHUR;
  }
  else if (solver_type == "ITERATIVE_SCHUR")
  {
    ROS_INFO("CeresSolver: Using ITERATIVE_SCHUR solver.");
    options_.linear_solver_type = ceres::ITERATIVE_SCHUR;
  }
  else if (solver_type == "CGNR")
  {
    ROS_INFO("CeresSolver: Using CGNR solver.");
    options_.linear_solver_type = ceres::CGNR;
  }

  // choose preconditioner default Jacobi
  options_.preconditioner_type = ceres::JACOBI;
  if (preconditioner_type == "IDENTITY")
  {
    ROS_INFO("CeresSolver: Using IDENTITY preconditioner.");
    options_.preconditioner_type = ceres::IDENTITY;
  }
  else if (preconditioner_type == "SCHUR_JACOBI")
  {
    ROS_INFO("CeresSolver: Using SCHUR_JACOBI preconditioner.");
    options_.preconditioner_type = ceres::SCHUR_JACOBI;
  }

  if (options_.preconditioner_type == ceres::CLUSTER_JACOBI || 
      options_.preconditioner_type == ceres::CLUSTER_TRIDIAGONAL)
  {
    //default canonical view is O(n^2) which is unacceptable for 
    // problems of this size
    options_.visibility_clustering_type = ceres::SINGLE_LINKAGE;
  }

  // choose trust region strategy default LM
  options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  if (trust_strategy == "DOGLEG")
  {
    ROS_INFO("CeresSolver: Using DOGLEG trust region strategy.");
    options_.trust_region_strategy_type = ceres::DOGLEG;
  }

  // choose dogleg type default traditional
  if(options_.trust_region_strategy_type == ceres::DOGLEG)
  {
    options_.dogleg_type = ceres::TRADITIONAL_DOGLEG;
    if (dogleg_type == "SUBSPACE_DOGLEG")
    {
      ROS_INFO("CeresSolver: Using SUBSPACE_DOGLEG dogleg type.");
      options_.dogleg_type = ceres::SUBSPACE_DOGLEG;
    }
  }

  // a typical ros map is 5cm, this is 0.001, 50x the resolution
  options_.function_tolerance = 1e-3;
  options_.gradient_tolerance = 1e-6;
  options_.parameter_tolerance = 1e-3; 

  options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options_.max_num_consecutive_invalid_steps = 3;
  options_.max_consecutive_nonmonotonic_steps =
    options_.max_num_consecutive_invalid_steps;
  options_.num_threads = 50;
  options_.use_nonmonotonic_steps = true;
  options_.jacobi_scaling = true;

  options_.min_relative_decrease = 1e-3;

  options_.initial_trust_region_radius = 1e4;
  options_.max_trust_region_radius = 1e8;
  options_.min_trust_region_radius = 1e-16;

  options_.min_lm_diagonal = 1e-6;
  options_.max_lm_diagonal = 1e32;

  if(options_.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY)
  {
    options_.dynamic_sparsity = true;
  }

  if (mode == std::string("localization"))
  {
    // doubles the memory footprint, but lets us remove contraints faster
    options_problem_.enable_fast_removal = true;
  }

  problem_ = new ceres::Problem(options_problem_);
}

/*****************************************************************************/
CeresSolver::~CeresSolver()
/*****************************************************************************/
{
  if (loss_function_ != NULL)
  {
    delete loss_function_;
  }
  if (residual_blocks_ != NULL)
  {
    delete residual_blocks_;
  }
  if (parameter_blocks_ != NULL)
  {
    delete parameter_blocks_;
  }
  if (problem_ != NULL)
  {
    delete problem_;  
  }
  if (local_parameterization_ != NULL)
  {
    delete local_parameterization_;
  }
}

/*****************************************************************************/
void CeresSolver::Compute()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mutex_);

  if (parameter_blocks_->empty())
  {
    ROS_ERROR("CeresSolver: Ceres was called when there are no nodes."
      " This shouldn't happen.");
    return;
  }

  // populate contraint for static initial pose
  if (!was_constant_set_ && first_parameter_block_ != nullptr)
  {
    ROS_DEBUG("CeresSolver: Setting node as a constant pose:"
      "%0.2f, %0.2f, %0.2f.", first_parameter_block_[0],
      first_parameter_block_[1], first_parameter_block_[2]);
    problem_->SetParameterBlockConstant(first_parameter_block_);
    was_constant_set_ = !was_constant_set_;
  }

  const ros::Time start_time = ros::Time::now();
  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);
  if (debug_logging_)
  {
    std::cout << summary.FullReport() << '\n';
  }

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
  corrections_.reserve(parameter_blocks_->size());

  karto::Pose2 pose;
  ParameterBlockMap::left_const_iterator it = parameter_blocks_->left.begin();
  for (; it != parameter_blocks_->left.end(); ++it)
  {
    pose.SetX(it->second[0]);
    pose.SetY(it->second[1]);
    pose.SetHeading(it->second[2]);
    corrections_.push_back(std::make_pair(it->first, pose));
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
Eigen::SparseMatrix<double> CeresSolver::GetInformationMatrix(
    std::unordered_map<int, Eigen::Index> * ordering) const
/****************************************************************************/
{
  if (ordering)
  {
    Eigen::Index index = 0u;
    std::vector<double*> parameter_blocks;
    problem_->GetParameterBlocks(&parameter_blocks);
    for (double * block : parameter_blocks)
    {
      (*ordering)[parameter_blocks_->right.at(block)] = index;
      index += problem_->ParameterBlockSize(block);
    }
  }
  ceres::CRSMatrix jacobian_data;
  problem_->Evaluate(ceres::Problem::EvaluateOptions(),
                     nullptr, nullptr, nullptr, &jacobian_data);
  Eigen::SparseMatrix<double> jacobian(
    problem_->NumResiduals(), problem_->NumParameters());
  jacobian.setFromTriplets(
    CRSMatrixIterator::begin(jacobian_data),
    CRSMatrixIterator::end(jacobian_data));
  return jacobian.transpose() * jacobian;
}

/*****************************************************************************/
void CeresSolver::Clear()
/*****************************************************************************/
{
  corrections_.clear();
}

/*****************************************************************************/
void CeresSolver::Reset()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mutex_);

  corrections_.clear();
  was_constant_set_ = false;

  if (problem_)
  {
    delete problem_;
  }

  if (local_parameterization_)
  {
    delete local_parameterization_;
  }

  if (parameter_blocks_)
  {
    delete parameter_blocks_;
  }

  if (residual_blocks_)
  {
    delete residual_blocks_;
  }

  parameter_block_pool_.purge_memory();

  parameter_blocks_ = new ParameterBlockMap();
  residual_blocks_ = new ResidualBlockMap();
  first_parameter_block_ = nullptr;

  problem_ = new ceres::Problem(options_problem_);
  local_parameterization_ = new ceres::ProductParameterization(
      new ceres::IdentityParameterization(2),
      AngleLocalParameterization::Create());
}

/*****************************************************************************/
void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
/*****************************************************************************/
{
  // store nodes
  if (!pVertex)
  {
    return;
  }

  boost::mutex::scoped_lock lock(mutex_);
  const int id = pVertex->GetObject()->GetUniqueId();
  const karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  double* parameter_block =
      static_cast<double *>(parameter_block_pool_.malloc());
  Eigen::Map<Eigen::Vector3d> parameter_block_as_vector(parameter_block);
  parameter_block_as_vector << pose.GetX(), pose.GetY(), pose.GetHeading();
  parameter_blocks_->insert(ParameterBlockMap::value_type(id, parameter_block));
  if (parameter_blocks_->size() == 1)
  {
    first_parameter_block_ = parameter_block;
  }
}

/*****************************************************************************/
void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
/*****************************************************************************/
{
  if (!pEdge)
  {
    return;
  }

  // get IDs in graph for this edge
  boost::mutex::scoped_lock lock(mutex_);

  ParameterBlockMap::left_const_iterator node1it =
    parameter_blocks_->left.find(
      pEdge->GetSource()->GetObject()->GetUniqueId());
  ParameterBlockMap::left_const_iterator node2it =
    parameter_blocks_->left.find(
      pEdge->GetTarget()->GetObject()->GetUniqueId());

  if (node1it == parameter_blocks_->left.end() ||
      node2it == parameter_blocks_->left.end() ||
      node1it == node2it)
  {
    ROS_WARN("CeresSolver: Failed to add constraint, could not find nodes.");
    return;
  }

  // extract transformation
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();

  Eigen::Matrix3d sqrt_information =
    pLinkInfo->GetCovariance().Inverse().ToEigen().llt().matrixU();
  // populate residual and parameterization for heading normalization
  ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
    diff.GetX(), diff.GetY(), diff.GetHeading(), sqrt_information);
  ceres::ResidualBlockId residual_block = problem_->AddResidualBlock(
    cost_function, loss_function_, node1it->second, node2it->second);
  problem_->SetParameterization(node1it->second, local_parameterization_);
  problem_->SetParameterization(node2it->second, local_parameterization_);

  residual_blocks_->emplace(
      GetHash(node1it->first, node1it->first), residual_block);
}

/*****************************************************************************/
void CeresSolver::RemoveNode(kt_int32s id)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mutex_);
  ParameterBlockMap::left_iterator it =
      parameter_blocks_->left.find(id);
  if (it != parameter_blocks_->left.end())
  {
    problem_->RemoveParameterBlock(it->second);
    parameter_block_pool_.free(it->second);
    if (it->second == first_parameter_block_)
    {
      it = parameter_blocks_->left.erase(it);
      if (it != parameter_blocks_->left.end())
      {
        first_parameter_block_ = it->second;
      }
      else
      {
        first_parameter_block_ = nullptr;
      }
      was_constant_set_ = false;
    }
    else
    {
      parameter_blocks_->left.erase(it);
    }
  }
  else
  {
    ROS_ERROR("RemoveNode: Failed to find node matching id %i", (int)id);
  }
}

/*****************************************************************************/
void CeresSolver::RemoveConstraint(kt_int32s sourceId, kt_int32s targetId)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mutex_);
  ResidualBlockMap::iterator it_a =
    residual_blocks_->find(GetHash(sourceId, targetId));
  ResidualBlockMap::iterator it_b =
    residual_blocks_->find(GetHash(targetId, sourceId));
  if (it_a != residual_blocks_->end())
  {
    problem_->RemoveResidualBlock(it_a->second);  
    residual_blocks_->erase(it_a);
  }
  else if (it_b != residual_blocks_->end())
  {
    problem_->RemoveResidualBlock(it_b->second);
    residual_blocks_->erase(it_b);
  }
  else
  {
    ROS_ERROR("RemoveConstraint: Failed to find residual block for %i %i", 
      (int)sourceId, (int)targetId);
  }
}

/*****************************************************************************/
void CeresSolver::ModifyNode(const int& unique_id, Eigen::Vector3d pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mutex_);
  ParameterBlockMap::left_iterator it =
    parameter_blocks_->left.find(unique_id);
  if (it != parameter_blocks_->left.end())
  {
    Eigen::Map<Eigen::Vector3d> parameter_block_as_vector(it->second);
    double yaw_init = parameter_block_as_vector(2);
    parameter_block_as_vector = pose;
    parameter_block_as_vector(2) += yaw_init;
  }
}

/*****************************************************************************/
void CeresSolver::GetNodeOrientation(const int& unique_id, double& pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mutex_);
  ParameterBlockMap::left_const_iterator it =
    parameter_blocks_->left.find(unique_id);
  if (it != parameter_blocks_->left.end())
  {
    pose = it->second[2];
  }
}

/*****************************************************************************/
std::unordered_map<int, Eigen::Map<Eigen::Vector3d>> CeresSolver::GetGraph()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(mutex_);
  std::unordered_map<int, Eigen::Map<Eigen::Vector3d>> graph;
  ParameterBlockMap::left_const_iterator it = parameter_blocks_->left.begin();
  for (; it != parameter_blocks_->left.end(); ++it)
  {
    graph.emplace(it->first, Eigen::Map<Eigen::Vector3d>(it->second));
  }
  return graph;
}

} // end namespace
