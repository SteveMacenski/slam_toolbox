/*
 * Copyright 2022 Ekumen Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2.1 of the License, or
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

#include "karto_sdk/contrib/ChowLiuTreeApprox.h"
#include "karto_sdk/contrib/EigenExtensions.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

namespace karto
{
  namespace contrib
  {

    Eigen::SparseMatrix<double> ComputeMarginalInformationMatrix(
        const Eigen::SparseMatrix<double> & information_matrix,
        const Eigen::Index discarded_variable_index,
        const Eigen::Index variables_dimension)
    {
      const Eigen::Index dimension = information_matrix.outerSize();
      assert(dimension == information_matrix.innerSize());  // must be square
      const Eigen::Index marginal_dimension = dimension - variables_dimension;
      const Eigen::Index last_variable_index = dimension - variables_dimension;
      // (1) Break up information matrix based on which are the variables
      // kept and which is the variable discarded (vectors `a` and `b` resp.).
      Eigen::SparseMatrix<double>
          information_submatrix_aa, information_submatrix_ab,
          information_submatrix_ba, information_submatrix_bb;
      if (discarded_variable_index == 0) {
        information_submatrix_aa =
            information_matrix.bottomRightCorner(
                marginal_dimension, marginal_dimension);
        information_submatrix_ab =
            information_matrix.bottomLeftCorner(
                marginal_dimension, variables_dimension);
        information_submatrix_ba =
            information_matrix.topRightCorner(
                variables_dimension, marginal_dimension);
        information_submatrix_bb =
            information_matrix.topLeftCorner(
                variables_dimension, variables_dimension);
      } else if (discarded_variable_index == last_variable_index) {
        information_submatrix_aa =
            information_matrix.topLeftCorner(
                marginal_dimension, marginal_dimension);
        information_submatrix_ab =
            information_matrix.topRightCorner(
                marginal_dimension, variables_dimension);
        information_submatrix_ba =
            information_matrix.bottomLeftCorner(
                variables_dimension, marginal_dimension);
        information_submatrix_bb =
            information_matrix.bottomRightCorner(
                variables_dimension, variables_dimension);
      } else {
        const Eigen::Index next_variable_index =
            discarded_variable_index + variables_dimension;
        information_submatrix_aa = StackVertically(
            StackHorizontally(
                information_matrix.topLeftCorner(
                    discarded_variable_index,
                    discarded_variable_index),
                information_matrix.topRightCorner(
                    discarded_variable_index,
                    dimension - next_variable_index)),
            StackHorizontally(
                information_matrix.bottomLeftCorner(
                    dimension - next_variable_index,
                    discarded_variable_index),
                information_matrix.bottomRightCorner(
                    dimension - next_variable_index,
                    dimension - next_variable_index)));
        information_submatrix_ab = StackVertically(
            information_matrix.block(
                0,
                discarded_variable_index,
                discarded_variable_index,
                variables_dimension),
            information_matrix.block(
                next_variable_index,
                discarded_variable_index,
                dimension - next_variable_index,
                variables_dimension));
        information_submatrix_ba = StackHorizontally(
            information_matrix.block(
                discarded_variable_index,
                0,
                variables_dimension,
                discarded_variable_index),
            information_matrix.block(
                discarded_variable_index,
                next_variable_index,
                variables_dimension,
                dimension - next_variable_index));
        information_submatrix_bb =
            information_matrix.block(
                discarded_variable_index,
                discarded_variable_index,
                variables_dimension,
                variables_dimension);
      }

      // (2) Compute generalized Schur's complement over the variables
      // that are kept.
      return (information_submatrix_aa - information_submatrix_ab *
              ComputeGeneralizedInverse(information_submatrix_bb) *
              information_submatrix_ba);
    }

    namespace {

      // An uncertain, gaussian-distributed 2D pose.
      struct UncertainPose2
      {
        Pose2 mean;
        Matrix3 covariance;
      };

      // Returns the target 2D pose relative to the source 2D pose,
      // accounting for their joint distribution covariance.
      UncertainPose2 ComputeRelativePose2(
          const Pose2 & source_pose, const Pose2 & target_pose,
          const Eigen::Matrix<double, 6, 6> & joint_pose_covariance)
      {
        // Computation is carried out as proposed in section 3.2 of:
        //
        //    R. Smith, M. Self and P. Cheeseman, "Estimating uncertain spatial
        //    relationships in robotics," Proceedings. 1987 IEEE International
        //    Conference on Robotics and Automation, 1987, pp. 850-850,
        //    doi: 10.1109/ROBOT.1987.1087846.
        //
        // In particular, this is a case of tail-tail composition of two spatial
        // relationships p_ij and p_ik as in: p_jk = ⊖ p_ij ⊕ p_ik
        UncertainPose2 relative_pose;
        // (1) Compute mean relative pose by simply
        // transforming mean source and target poses.
        Transform source_transform(source_pose);
        relative_pose.mean =
            source_transform.InverseTransformPose(target_pose);
        // (2) Compute relative pose covariance by linearizing
        // the transformation around mean source and target
        // poses.
        Eigen::Matrix<double, 3, 6> transform_jacobian;
        const double x_jk = relative_pose.mean.GetX();
        const double y_jk = relative_pose.mean.GetY();
        const double theta_ij = source_pose.GetHeading();
        transform_jacobian <<
            -cos(theta_ij), -sin(theta_ij),  y_jk,  cos(theta_ij), sin(theta_ij), 0.0,
             sin(theta_ij), -cos(theta_ij), -x_jk, -sin(theta_ij), cos(theta_ij), 0.0,
                       0.0,            0.0,  -1.0,            0.0,           0.0, 1.0;
        const Eigen::Matrix3d covariance =
            transform_jacobian * joint_pose_covariance * transform_jacobian.transpose();
        assert(covariance.isApprox(covariance.transpose()));  // must be symmetric
        assert((covariance.array() > 0.).all());  // must be positive semidefinite
        relative_pose.covariance = Matrix3(covariance);
        return relative_pose;
      }

    }  // namespace

    std::vector<Edge<LocalizedRangeScan> *> ComputeChowLiuTreeApproximation(
        const std::vector<Vertex<LocalizedRangeScan> *> & clique,
        const Eigen::MatrixXd & covariance_matrix)
    {
      // (1) Build clique subgraph, weighting edges by the *negated* mutual
      // information between corresponding variables (so as to apply
      // Kruskal's minimum spanning tree algorithm down below).
      using WeightedGraphT = boost::adjacency_list<
        boost::vecS, boost::vecS, boost::undirectedS, boost::no_property,
        boost::property<boost::edge_weight_t, double>>;
      using VertexDescriptorT =
          boost::graph_traits<WeightedGraphT>::vertex_descriptor;
      WeightedGraphT clique_subgraph(clique.size());
      for (VertexDescriptorT u = 0; u < clique.size() - 1; ++u) {
        for (VertexDescriptorT v = u + 1; v < clique.size(); ++v) {
          const Eigen::Index i = u * 3, j = v * 3;  // need block indices
          const auto covariance_submatrix_ii =
              covariance_matrix.block(i, i, 3, 3);
          const auto covariance_submatrix_ij =
              covariance_matrix.block(i, j, 3, 3);
          const auto covariance_submatrix_ji =
              covariance_matrix.block(j, i, 3, 3);
          const auto covariance_submatrix_jj =
              covariance_matrix.block(j, j, 3, 3);
          const double mutual_information =
              0.5 * std::log2(covariance_submatrix_ii.determinant() / (
                  covariance_submatrix_ii - covariance_submatrix_ij *
                  ComputeGeneralizedInverse(covariance_submatrix_jj) *
                  covariance_submatrix_ji).determinant());
          boost::add_edge(u, v, -mutual_information, clique_subgraph);
        }
      }
      // (2) Find maximum mutual information spanning tree in the clique subgraph
      // (which best approximates the underlying joint probability distribution as
      // proved by Chow & Liu).
      using EdgeDescriptorT =
          boost::graph_traits<WeightedGraphT>::edge_descriptor;
      std::vector<EdgeDescriptorT> minimum_spanning_tree_edges;
      boost::kruskal_minimum_spanning_tree(
          clique_subgraph, std::back_inserter(minimum_spanning_tree_edges));
      // (3) Build tree approximation as an edge list, using the mean and
      // covariance of the marginal joint distribution between each variable
      // to recompute the nonlinear constraint (i.e. a 2D isometry) between them.
      std::vector<Edge<LocalizedRangeScan> *> chow_liu_tree_approximation;
      for (const EdgeDescriptorT & edge_descriptor : minimum_spanning_tree_edges) {
        const VertexDescriptorT u = boost::source(edge_descriptor, clique_subgraph);
        const VertexDescriptorT v = boost::target(edge_descriptor, clique_subgraph);
        auto * edge = new Edge<LocalizedRangeScan>(clique[u], clique[v]);
        const Eigen::Index i = u * 3, j = v * 3;  // need block indices
        Eigen::Matrix<double, 6, 6> joint_pose_covariance_matrix;
        joint_pose_covariance_matrix <<  // marginalized from the larger matrix
            covariance_matrix.block(i, i, 3, 3), covariance_matrix.block(i, j, 3, 3),
            covariance_matrix.block(j, i, 3, 3), covariance_matrix.block(j, j, 3, 3);
        LocalizedRangeScan * source_scan = edge->GetSource()->GetObject();
        LocalizedRangeScan * target_scan = edge->GetTarget()->GetObject();
        const UncertainPose2 relative_pose =
            ComputeRelativePose2(source_scan->GetCorrectedPose(),
                                 target_scan->GetCorrectedPose(),
                                 joint_pose_covariance_matrix);
        // TODO(hidmic): figure out how to handle rank deficient constraints
        assert(relative_pose.covariance.ToEigen().fullPivLu().rank() == 3);
        edge->SetLabel(new LinkInfo(
            source_scan->GetCorrectedPose(),
            target_scan->GetCorrectedPose(),
            relative_pose.mean, relative_pose.covariance));
        chow_liu_tree_approximation.push_back(edge);
      }
      return chow_liu_tree_approximation;
    }

  }  // namespace contrib
}  // namespace karto
