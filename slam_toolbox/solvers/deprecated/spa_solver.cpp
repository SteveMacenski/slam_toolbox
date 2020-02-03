/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "spa_solver.hpp"
#include <karto_sdk/Karto.h>

#include "ros/console.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(solver_plugins::SpaSolver, karto::ScanSolver)

namespace solver_plugins
{
typedef std::vector<sba::Node2d, Eigen::aligned_allocator<sba::Node2d> > NodeVector;

/*****************************************************************************/
SpaSolver::SpaSolver()
/*****************************************************************************/
{

}

/*****************************************************************************/
SpaSolver::~SpaSolver()
/*****************************************************************************/
{

}

/*****************************************************************************/
void SpaSolver::Clear()
/*****************************************************************************/
{
  corrections.clear();
}

const karto::ScanSolver::IdPoseVector& SpaSolver::GetCorrections() const
{
  return corrections;
}

/*****************************************************************************/
void SpaSolver::Compute()
/*****************************************************************************/
{
  corrections.clear();

  const ros::Time start_time = ros::Time::now();
  m_Spa.doSPA(40, 1.0e-4, 1);
  ROS_INFO("Loop Closure Solve time: %f seconds",
    (ros::Time::now() - start_time).toSec());

  NodeVector nodes = m_Spa.getNodes();
  forEach(NodeVector, &nodes)
  {
    karto::Pose2 pose(iter->trans(0), iter->trans(1), iter->arot);
    corrections.push_back(std::make_pair(iter->nodeId, pose));
  }
}

/*****************************************************************************/
void SpaSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
/*****************************************************************************/
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
  m_Spa.addNode(vector, pVertex->GetObject()->GetUniqueId());
}

/*****************************************************************************/
void SpaSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
/*****************************************************************************/
{
  karto::LocalizedRangeScan* pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan* pTarget = pEdge->GetTarget()->GetObject();
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  Eigen::Vector3d mean(diff.GetX(), diff.GetY(), diff.GetHeading());

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double,3,3> m;
  m(0,0) = precisionMatrix(0,0);
  m(0,1) = m(1,0) = precisionMatrix(0,1);
  m(0,2) = m(2,0) = precisionMatrix(0,2);
  m(1,1) = precisionMatrix(1,1);
  m(1,2) = m(2,1) = precisionMatrix(1,2);
  m(2,2) = precisionMatrix(2,2);

  m_Spa.addConstraint(pSource->GetUniqueId(), pTarget->GetUniqueId(), mean, m);
}

/*****************************************************************************/
void SpaSolver::getGraph(std::vector<Eigen::Vector2d>& g)
/*****************************************************************************/
{
  std::vector<float> raw_graph;
  m_Spa.getGraph(raw_graph);

  g.reserve(raw_graph.size()/4);

  Eigen::Vector2d pose;
  for (size_t i=0; i!=raw_graph.size()/4; i++)
  {
    pose(0) = raw_graph[4*i];
    pose(1) = raw_graph[4*i+1];
    g.push_back(pose);
  }
}


} // end namespace