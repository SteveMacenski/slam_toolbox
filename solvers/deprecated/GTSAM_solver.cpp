/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

/* Authors: Saurav Agarwal */
/* Modified: Steve Macenski */

#include <limits>
#include <karto_sdk/Karto.h>
#include <ros/console.h>
#include "GTSAM_solver.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(solver_plugins::GTSAMSolver, karto::ScanSolver)

namespace solver_plugins
{

/*****************************************************************************/
GTSAMSolver::GTSAMSolver()
/*****************************************************************************/
{
  using namespace gtsam;

  // add the prior on the first node which is known
  noiseModel::Diagonal::shared_ptr priorNoise = 
    noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));
  
  graph_.emplace_shared<PriorFactor<Pose2> >(0, Pose2(0, 0, 0), priorNoise);

}

/*****************************************************************************/
GTSAMSolver::~GTSAMSolver()
/*****************************************************************************/
{
  
}

/*****************************************************************************/
void GTSAMSolver::Clear()
/*****************************************************************************/
{
  corrections_.clear();
}

/*****************************************************************************/
const karto::ScanSolver::IdPoseVector& GTSAMSolver::GetCorrections() const
/*****************************************************************************/
{
  return corrections_;
}

/*****************************************************************************/
void GTSAMSolver::Compute()
/*****************************************************************************/
{
  using namespace gtsam;

  corrections_.clear();

  graphNodes_.clear();

  LevenbergMarquardtParams parameters;

  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;
  
  // Do not perform more than N iteration steps
  parameters.maxIterations = 500;
  
  // Create the optimizer ...
  LevenbergMarquardtOptimizer optimizer(graph_, initialGuess_, parameters);
  
  // ... and optimize
  Values result = optimizer.optimize();

  Values::ConstFiltered<Pose2> viewPose2 = result.filter<Pose2>();
  
  // put values into corrections container
  for(const Values::ConstFiltered<Pose2>::KeyValuePair& key_value: viewPose2) 
  {

    karto::Pose2 pose(key_value.value.x(), key_value.value.y(),
      key_value.value.theta());
    
    corrections_.push_back(std::make_pair(key_value.key, pose));

    graphNodes_.push_back(Eigen::Vector2d(key_value.value.x(),
      key_value.value.y()));
  }
}

/*****************************************************************************/
void GTSAMSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
/*****************************************************************************/
{
  using namespace gtsam;

  karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();
  
  initialGuess_.insert(pVertex->GetObject()->GetUniqueId(), 
                        Pose2( odom.GetX(), odom.GetY(), odom.GetHeading() ));

  graphNodes_.push_back(Eigen::Vector2d(odom.GetX(), odom.GetY()));
  
  ROS_DEBUG("[gtsam] Adding node %d.", pVertex->GetObject()->GetUniqueId());

}

/*****************************************************************************/
void GTSAMSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
/*****************************************************************************/
{
  using namespace gtsam;

  // Set source and target
  int sourceID = pEdge->GetSource()->GetObject()->GetUniqueId();
  
  int targetID = pEdge->GetTarget()->GetObject()->GetUniqueId();
  
  // Set the measurement (poseGraphEdge distance between vertices)
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());
  
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  
  // Set the covariance of the measurement
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance();
  
  Eigen::Matrix<double,3,3> cov;
  
  cov(0,0) = precisionMatrix(0,0);
  
  cov(0,1) = cov(1,0) = precisionMatrix(0,1);
  
  cov(0,2) = cov(2,0) = precisionMatrix(0,2);
  
  cov(1,1) = precisionMatrix(1,1);
  
  cov(1,2) = cov(2,1) = precisionMatrix(1,2);
  
  cov(2,2) = precisionMatrix(2,2);
  
  noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Covariance(cov);

  // Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  graph_.emplace_shared<BetweenFactor<Pose2> >(sourceID, targetID,
    Pose2(diff.GetX(), diff.GetY(), diff.GetHeading()), model);
  
  // Add the constraint to the optimizer
  ROS_DEBUG("[gtsam] Adding Edge from node %d to node %d.", sourceID, targetID);
  
}

/*****************************************************************************/
void GTSAMSolver::getGraph(std::vector<Eigen::Vector2d>& nodes)
/*****************************************************************************/
{
  nodes = graphNodes_;
  // using namespace gtsam;
  // double *data1 = new double[3];
  // double *data2 = new double[3];
  // for (SparseOptimizer::EdgeSet::iterator it = 
  //    optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it) 
  // {
  //   EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
  //   if(e) 
  //   {
  //     VertexSE2* v1 = dynamic_cast<VertexSE2*>(e->vertices()[0]);
  //     v1->getEstimateData(data1);
  //     Eigen::Vector2d poseFrom(data1[0], data1[1]);
  //     VertexSE2* v2 = dynamic_cast<VertexSE2*>(e->vertices()[1]);
  //     v2->getEstimateData(data2);
  //     Eigen::Vector2d poseTo(data2[0], data2[1]);
  //     edges.push_back(std::make_pair(poseFrom, poseTo));
  //   }
  // }
  // delete data1;
  // delete data2;
}

} // end namespace