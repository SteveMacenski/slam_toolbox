/*
 * Copyright 2018 Simbe Robotics
 * Author: Steve Macenski
 */

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>
#include <iterator>
#include <utility>

#include <Eigen/SparseCore>

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline std::size_t GetHash(const int& x, const int& y)
{
  return ((std::hash<double>()(x) ^ (std::hash<double>()(y) << 1)) >> 1);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Iterator for ceres::CRSMatrix elements that yields Eigen::Triplet instances.
// Helpful to populate Eigen::SparseMatrix instances from ceres::CRSMatrix ones.
class CRSMatrixIterator : public std::iterator<
  std::input_iterator_tag, Eigen::Triplet<double>>
{
public:
  static CRSMatrixIterator begin(const ceres::CRSMatrix & matrix)
  {
    return CRSMatrixIterator(matrix);
  }

  static CRSMatrixIterator end(const ceres::CRSMatrix & matrix)
  {
    return CRSMatrixIterator(matrix, matrix.num_rows);
  }

  CRSMatrixIterator& operator++()
  {
    if (++data_index_ == matrix_.rows[row_index_ + 1])
    {
      ++row_index_;
    }
    current_triplet_ = MakeTriplet();
    return *this;
  }

  CRSMatrixIterator operator++(int)
  {
    CRSMatrixIterator it = *this;
    ++(*this);
    return it;
  }

  bool operator==(const CRSMatrixIterator & other) const
  {
    return &matrix_ == &other.matrix_ &&
        row_index_ == other.row_index_ &&
        data_index_ == other.data_index_;
  }

  bool operator!=(const CRSMatrixIterator & other) const
  {
    return !(*this == other);
  }

  pointer operator->() {
    return &current_triplet_;
  }

  reference operator*() {
    return current_triplet_;
  }

 private:
  explicit CRSMatrixIterator(
      const ceres::CRSMatrix & matrix,
      size_t row_index = 0)
  : matrix_(matrix),
    row_index_(row_index),
    data_index_(matrix.rows[row_index])
  {
    current_triplet_ = MakeTriplet();
  }

  Eigen::Triplet<double> MakeTriplet() const
  {
    return Eigen::Triplet<double>(
      row_index_, matrix_.cols[data_index_],
      matrix_.values[data_index_]);
  }

  const ceres::CRSMatrix & matrix_;
  size_t row_index_;
  size_t data_index_;

  Eigen::Triplet<double> current_triplet_;
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Normalizes the angle in radians between [-pi and pi).
template <typename T> inline T NormalizeAngle(const T& angle_radians)
{
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class AngleLocalParameterization
{
 public:
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const
  {
    *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization* Create()
  {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
  }
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
{
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);
  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class PoseGraph2dErrorTerm
{
 public:
  PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians, const Eigen::Matrix3d& sqrt_information)
      : p_ab_(x_ab, y_ab), yaw_ab_radians_(yaw_ab_radians), sqrt_information_(sqrt_information)
  {
  }

  template <typename T>
  bool operator()(const T* pose_a, const T* pose_b, T* residuals_ptr) const
  {
    const Eigen::Matrix<T, 2, 1> p_a(pose_a);
    const Eigen::Matrix<T, 2, 1> p_b(pose_b);
    const T yaw_a = pose_a[2];
    const T yaw_b = pose_b[2];

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);

    residuals.template head<2>() =
        RotationMatrix2D(yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
    residuals(2) = NormalizeAngle((yaw_b - yaw_a) - static_cast<T>(yaw_ab_radians_));
    // Scale the residuals by the square root information matrix to account for the measurement uncertainty.
    residuals = sqrt_information_.template cast<T>() * residuals;
    return true;
  }

  static ceres::CostFunction* Create(double x_ab, double y_ab, double yaw_ab_radians, const Eigen::Matrix3d& sqrt_information) 
  {
    return (new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 3, 3>(new PoseGraph2dErrorTerm(x_ab, y_ab, yaw_ab_radians, sqrt_information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

