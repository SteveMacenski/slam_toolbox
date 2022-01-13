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

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "karto_sdk/contrib/EigenExtensions.h"

namespace
{

using namespace karto::contrib;

TEST(EigenSparseExtensionsTest, HorizontalStacking)
{
  Eigen::MatrixXd matrix(3, 4);
  matrix << 1., 0., 3., 0.,
            0., 0., 6., 2.,
            7., 8., 0., 0.;

  Eigen::MatrixXd duplicated_matrix(3, 8);
  duplicated_matrix << 1., 0., 3., 0., 1., 0., 3., 0.,
                       0., 0., 6., 2., 0., 0., 6., 2.,
                       7., 8., 0., 0., 7., 8., 0., 0.;
  EXPECT_TRUE(duplicated_matrix.sparseView().isApprox(
      StackHorizontally(matrix.sparseView(),
                        matrix.sparseView())));

  Eigen::MatrixXd duplicated_row(1, 8);
  duplicated_row << 1., 0., 3., 0., 1., 0., 3., 0.;
  EXPECT_TRUE(duplicated_row.sparseView().isApprox(
      StackHorizontally(matrix.sparseView().row(0),
                        matrix.sparseView().row(0))));

  Eigen::MatrixXd duplicated_col(3, 2);
  duplicated_col << 1., 1.,
                    0., 0.,
                    7., 7.;
  EXPECT_TRUE(duplicated_col.sparseView().isApprox(
      StackHorizontally(matrix.sparseView().col(0),
                        matrix.sparseView().col(0))));
}

TEST(EigenSparseExtensionsTest, VerticalStacking)
{
  Eigen::MatrixXd matrix(3, 4);
  matrix << 1., 0., 3., 0.,
            0., 0., 6., 2.,
            7., 8., 0., 0.;

  Eigen::MatrixXd duplicated_matrix(6, 4);
  duplicated_matrix << 1., 0., 3., 0.,
                       0., 0., 6., 2.,
                       7., 8., 0., 0.,
                       1., 0., 3., 0.,
                       0., 0., 6., 2.,
                       7., 8., 0., 0.;
  EXPECT_TRUE(duplicated_matrix.sparseView().isApprox(
      StackVertically(matrix.sparseView(),
                      matrix.sparseView())));

  Eigen::MatrixXd duplicated_row(2, 4);
  duplicated_row << 1., 0., 3., 0.,
                    1., 0., 3., 0.;
  EXPECT_TRUE(duplicated_row.sparseView().isApprox(
      StackVertically(matrix.sparseView().row(0),
                      matrix.sparseView().row(0))));

  Eigen::MatrixXd duplicated_col(6, 1);
  duplicated_col << 1.,
                    0.,
                    7.,
                    1.,
                    0.,
                    7.;
  EXPECT_TRUE(duplicated_col.sparseView().isApprox(
      StackVertically(matrix.sparseView().col(0),
                      matrix.sparseView().col(0))));
}

TEST(EigenSparseExtensionsTest, RearrangedView)
{
  Eigen::MatrixXd matrix(3, 4);
  matrix << 1., 0., 3., 0.,
            0., 0., 6., 2.,
            7., 8., 0., 0.;
  Eigen::MatrixXd rearranged_submatrix(2, 3);
  rearranged_submatrix << 8., 0., 7.,
                          0., 0., 1.;

  EXPECT_TRUE(rearranged_submatrix.sparseView().isApprox(
      ArrangeView(matrix.sparseView(), {2, 0}, {1, 3, 0})));
}

TEST(EigenSparseExtensionsTest, GeneralizedInverse)
{
  Eigen::MatrixXd invertible_matrix(3, 3);
  invertible_matrix << 1., 2., -3.,
                       2., 1.,  2.,
                      -3., 2.,  1.;
  EXPECT_TRUE(invertible_matrix.inverse().isApprox(
      ComputeGeneralizedInverse(invertible_matrix)));

  Eigen::MatrixXd singular_matrix(3, 3);
  singular_matrix << 1.,  2., 0.,
                     2., -1., 0.,
                     0.,  0., 0.;
  Eigen::MatrixXd singular_matrix_pseudoinverse(3, 3);
  singular_matrix_pseudoinverse << 0.2,  0.4, 0.,
                                   0.4, -0.2, 0.,
                                   0.0,  0.0, 0.;
  EXPECT_TRUE(singular_matrix_pseudoinverse.isApprox(
      ComputeGeneralizedInverse(singular_matrix)));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
