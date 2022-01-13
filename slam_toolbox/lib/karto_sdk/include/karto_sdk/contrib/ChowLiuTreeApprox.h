/*
 * Copyright 2022 Ekumen Inc.
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

#ifndef KARTO_SDK__CHOW_LIU_TREE_APPROX_H_
#define KARTO_SDK__CHOW_LIU_TREE_APPROX_H_

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "karto_sdk/Karto.h"
#include "karto_sdk/Mapper.h"
#include "karto_sdk/Types.h"

namespace karto
{
  namespace contrib
  {

    /** Marginalizes a variable from a sparse information matrix. */
    Eigen::SparseMatrix<double> ComputeMarginalInformationMatrix(
        const Eigen::SparseMatrix<double> & information_matrix,
        const Eigen::Index discarded_variable_index,
        const Eigen::Index variables_dimension);

    /**
     * Computes a Chow Liu tree approximation to a given pose graph clique.
     *
     * Currently, this function only performs linear approximations to full
     * rank constraints (i.e. constraints with full rank covariance matrices).
     */
    std::vector<Edge<LocalizedRangeScan> *> ComputeChowLiuTreeApproximation(
        const std::vector<Vertex<LocalizedRangeScan> *> & clique,
        const Eigen::MatrixXd & covariance_matrix);

  }  // namespace contrib
}  // namespace karto

#endif // KARTO_SDK__CHOW_LIU_TREE_APPROX_H_
