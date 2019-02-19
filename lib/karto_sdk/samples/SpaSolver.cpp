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

#include "SpaSolver.h"
#include <OpenKarto/Karto.h>

SpaSolver::SpaSolver()
{

}

SpaSolver::~SpaSolver()
{

}

void SpaSolver::Clear()
{
  corrections.clear();
}

const karto::ScanSolver::IdPoseVector& SpaSolver::GetCorrections() const
{
  return corrections;
}

void SpaSolver::Compute()
{
  corrections.clear();

  typedef std::vector<sba::Node2d, Eigen::aligned_allocator<sba::Node2d> > NodeVector;

  printf("DO SPA BEGIN\n");
  m_Spa.doSPA(40);
  printf("DO SPA END\n");
  NodeVector nodes = m_Spa.getNodes();
  forEach(NodeVector, &nodes)
  {
    karto::Pose2 pose(iter->trans(0), iter->trans(1), iter->arot);
    corrections.push_back(std::make_pair(iter->nodeId, pose));
  }
}

void SpaSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
  m_Spa.addNode2d(vector, pVertex->GetObject()->GetUniqueId());
}

void SpaSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
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

  m_Spa.addConstraint2d(pSource->GetUniqueId(), pTarget->GetUniqueId(), mean, m);
}
