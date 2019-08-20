/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

/* Authors: Saurav Agarwal */
/* Modified: Steve Macenski */

#ifndef KARTO_GTSAMSolver_H
#define KARTO_GTSAMSolver_H

#include <karto_sdk/Mapper.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace solver_plugins
{

/**
 * @brief Wrapper for G2O to interface with Open Karto
 */
class GTSAMSolver : public karto::ScanSolver
{
  public:

    GTSAMSolver();
    
    virtual ~GTSAMSolver();

  public:
    
    /**
     * @brief Clear the vector of corrections
     * @details Empty out previously computed corrections
     */
    virtual void Clear();
    
    /**
     * @brief Solve the SLAM back-end
     * @details Calls G2O to solve the SLAM back-end
     */
    virtual void Compute();
    
    /**
     * @brief Get the vector of corrections
     * @details Get the vector of corrections
     * @return Vector with corrected poses
     */
    virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

    /**
     * @brief Add a node to pose-graph
     * @details Add a node which is a robot pose to the pose-graph
     * 
     * @param pVertex the node to be added in
     */
    virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
    
    /**
     * @brief Add an edge constraint to pose-graph
     * @details Adds a relative pose measurement constraint between two poses in the graph
     * 
     * @param pEdge [description]
     */
    virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

    /**
     * @brief Get the pose-graph 
     * @details Get the underlying graph from g2o, return the graph of constraints
     * 
     * @param g the graph
     */
    virtual void getGraph(std::vector<Eigen::Vector2d>& nodes); // std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges);

    virtual void ModifyNode(const int& unique_id, const Eigen::Vector3d& pose); // change a node's pose

  private:
    
    karto::ScanSolver::IdPoseVector corrections_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initialGuess_;
    std::vector<Eigen::Vector2d> graphNodes_;
};

} // end namespace 

#endif // KARTO_GTSAMSolver_H

