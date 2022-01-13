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
#include <Eigen/Sparse>

#include "karto_sdk/contrib/ChowLiuTreeApprox.h"

namespace
{

using namespace karto;
using namespace karto::contrib;

TEST(ChowLiuTreeApproxTest, Marginalization)
{
  constexpr Eigen::Index block_size = 2;
  Eigen::MatrixXd information_matrix(6, 6);
  information_matrix <<
      1.0, 0.0, 0., 0., 0.5, 0.0,
      0.0, 1.0, 0., 0., 0.0, 0.5,
      0.0, 0.0, 1., 0., 0.0, 0.0,
      0.0, 0.0, 0., 1., 0.0, 0.0,
      0.5, 0.0, 0., 0., 1.0, 0.0,
      0.0, 0.5, 0., 0., 0.0, 1.0;

  Eigen::MatrixXd left_marginal(4, 4);
  left_marginal <<
      0.75, 0.0,  0., 0.,
      0.0,  0.75, 0., 0.,
      0.0,  0.0,  1., 0.,
      0.0,  0.0,  0., 1.;
  constexpr Eigen::Index right_most_block_index = 4;
  EXPECT_TRUE(left_marginal.sparseView().isApprox(
      ComputeMarginalInformationMatrix(
          information_matrix.sparseView(),
          right_most_block_index, block_size)));

  Eigen::MatrixXd right_marginal(4, 4);
  right_marginal <<
      1., 0.,  0.,   0.,
      0., 1.,  0.,   0.,
      0., 0.,  0.75, 0.,
      0., 0.,  0.,   0.75;
  constexpr Eigen::Index left_most_block_index = 0;
  EXPECT_TRUE(right_marginal.sparseView().isApprox(
      ComputeMarginalInformationMatrix(
          information_matrix.sparseView(),
          left_most_block_index, block_size)));

  Eigen::MatrixXd outer_marginal(4, 4);
  outer_marginal <<
      1.0, 0.0, 0.5, 0.0,
      0.0, 1.0, 0.0, 0.5,
      0.5, 0.0, 1.0, 0.0,
      0.0, 0.5, 0.0, 1.0;
  constexpr Eigen::Index inner_block_index = 2;
  EXPECT_TRUE(outer_marginal.sparseView().isApprox(
      ComputeMarginalInformationMatrix(
          information_matrix.sparseView(),
          inner_block_index, block_size)));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
