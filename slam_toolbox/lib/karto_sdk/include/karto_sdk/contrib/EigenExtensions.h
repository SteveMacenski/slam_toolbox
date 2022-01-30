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

#ifndef KARTO_SDK__EIGEN_EXTENSIONS_H_
#define KARTO_SDK__EIGEN_EXTENSIONS_H_

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SVD>

namespace Eigen {

namespace internal {

// Returns (a + b) or Dynamic if either operand is.
template<typename A, typename B>
inline constexpr int size_sum_prefer_dynamic(A a, B b) {
  static_assert(
      std::is_enum<A>::value || std::is_integral<A>::value,
      "Argument a must be an integer or enum");
  static_assert(
      std::is_enum<B>::value || std::is_integral<B>::value,
      "Argument b must be an integer or enum");
  if ((int) a == Dynamic || (int) b == Dynamic) return Dynamic;
  return (int) a + (int) b;
}

// Returns max(a, b) or Dynamic if either operand is.
template<typename A, typename B>
inline constexpr int max_size_prefer_dynamic(A a, B b) {
  static_assert(
      std::is_enum<A>::value || std::is_integral<A>::value,
      "Argument a must be an integer or enum");
  static_assert(
      std::is_enum<B>::value || std::is_integral<B>::value,
      "Argument b must be an integer or enum");
  if ((int) a == Dynamic || (int) b == Dynamic) return Dynamic;
  return std::max((int) a, (int) b);
}

template<bool Condition, typename ThenT, typename ElseT>
struct constexpr_conditional_impl;

template<typename ThenT, typename ElseT>
struct constexpr_conditional_impl<true, ThenT, ElseT> {
  constexpr_conditional_impl(ThenT&& some_value, ElseT&&)
    : value(some_value)
  {
  }

  ThenT value;
};

template<typename ThenT, typename ElseT>
struct constexpr_conditional_impl<false, ThenT, ElseT> {
  constexpr_conditional_impl(ThenT&&, ElseT&& other_value)
    : value(other_value)
  {
  }

  ElseT value;
};

// Returns `some_value` if `Condition`, else `other_value`.
//
// Compile-time if-then-else expression where `some_value`
// and `other_value` types need not match.
template<bool Condition, typename ThenT, typename ElseT>
inline constexpr auto
constexpr_conditional(ThenT&& some_value, ElseT&& other_value)
{
  return constexpr_conditional_impl<Condition, ThenT, ElseT>{
    std::forward<ThenT>(some_value),
    std::forward<ElseT>(other_value)}.value;
}

}  // namespace internal

// Forward declaration.
template<typename LhsType, typename RhsType>
class HorizontalStack;

// Forward declaration.
template<typename LhsType, typename RhsType>
class VerticalStack;

// Forward declaration.
template<typename XprType, typename RowIndices, typename ColIndices>
class View;

namespace internal {

template <typename A, typename B>
struct promote_storage_kind;

template <typename A>
struct promote_storage_kind<A, Sparse> { using type = Sparse; };

template <typename B>
struct promote_storage_kind<Sparse, B> { using type = Sparse; };

template <>
struct promote_storage_kind<Sparse, Sparse> { using type = Sparse; };

template <typename A, typename B>
struct promote_scalar {
  static_assert(
      std::is_convertible<A, B>::value ||
      std::is_convertible<B, A>::value,
      "Scalar types are incommensurable");

  using type = typename std::conditional<
    std::is_convertible<A, B>::value, B, A>::type;
};

template<typename LhsType, typename RhsType>
struct traits<HorizontalStack<LhsType, RhsType>>
{
  using XprKind = typename traits<LhsType>::XprKind;
  using StorageKind = typename promote_storage_kind<
    typename traits<LhsType>::StorageKind,
    typename traits<RhsType>::StorageKind>::type;
  using StorageIndex = typename promote_index_type<
    typename traits<LhsType>::StorageIndex,
    typename traits<RhsType>::StorageIndex>::type;
  using Scalar = typename promote_scalar<
    typename traits<LhsType>::Scalar,
    typename traits<RhsType>::Scalar>::type;
  enum {
    RowsAtCompileTime = (
        traits<LhsType>::RowsAtCompileTime == Dynamic ?
        traits<RhsType>::RowsAtCompileTime :
        traits<LhsType>::RowsAtCompileTime),
    ColsAtCompileTime = internal::size_sum_prefer_dynamic(
        traits<LhsType>::ColsAtCompileTime,
        traits<RhsType>::ColsAtCompileTime),
    MaxRowsAtCompileTime = internal::max_size_prefer_dynamic(
        traits<LhsType>::MaxRowsAtCompileTime,
        traits<RhsType>::MaxRowsAtCompileTime),
    MaxColsAtCompileTime = internal::size_sum_prefer_dynamic(
        traits<LhsType>::MaxColsAtCompileTime,
        traits<RhsType>::MaxColsAtCompileTime),
    Flags = int(traits<LhsType>::Flags) & RowMajorBit
  };
};

template<typename LhsType, typename RhsType>
struct traits<VerticalStack<LhsType, RhsType>>
{
  using XprKind = typename traits<LhsType>::XprKind;
  using StorageKind = typename promote_storage_kind<
    typename traits<LhsType>::StorageKind,
    typename traits<RhsType>::StorageKind>::type;
  using StorageIndex = typename promote_index_type<
    typename traits<LhsType>::StorageIndex,
    typename traits<RhsType>::StorageIndex>::type;
  using Scalar = typename promote_scalar<
    typename traits<LhsType>::Scalar,
    typename traits<RhsType>::Scalar>::type;
  enum {
    RowsAtCompileTime = internal::size_sum_prefer_dynamic(
        traits<LhsType>::RowsAtCompileTime,
        traits<RhsType>::RowsAtCompileTime),
    ColsAtCompileTime = (
        traits<LhsType>::ColsAtCompileTime == Dynamic ?
        traits<RhsType>::ColsAtCompileTime :
        traits<LhsType>::ColsAtCompileTime),
    MaxRowsAtCompileTime = internal::size_sum_prefer_dynamic(
        traits<LhsType>::MaxRowsAtCompileTime,
        traits<RhsType>::MaxRowsAtCompileTime),
    MaxColsAtCompileTime = internal::max_size_prefer_dynamic(
        traits<LhsType>::MaxColsAtCompileTime,
        traits<RhsType>::MaxColsAtCompileTime),
    Flags = int(traits<LhsType>::Flags) & RowMajorBit
  };
};

template<typename XprType, typename RowIndices, typename ColIndices>
struct traits<View<XprType, RowIndices, ColIndices>> : traits<XprType>
{
  enum {
    RowsAtCompileTime = Dynamic,
    ColsAtCompileTime = Dynamic,
    MaxRowsAtCompileTime = RowsAtCompileTime,
    MaxColsAtCompileTime = ColsAtCompileTime,
    IsRowMajor = (int(traits<XprType>::Flags) & RowMajorBit) != 0,
    FlagsRowMajorBit = IsRowMajor ? RowMajorBit : 0,
    Flags = int(traits<XprType>::Flags) & RowMajorBit,
  };
};

}  // namespace internal

/**
 * Expression of a column by column concatenation (i.e. horizontal stacking)
 * of two matrix (or array) expressions.
 *
 * Only sparse expressions are supported.
 */
template<typename LhsType, typename RhsType>
class HorizontalStack : public internal::generic_xpr_base<
  HorizontalStack<LhsType, RhsType>>::type, internal::no_assignment_operator
{
 public:
  EIGEN_STATIC_ASSERT_SAME_XPR_KIND(LhsType, RhsType)
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(LhsType, RhsType)
  EIGEN_STATIC_ASSERT((
      ((internal::traits<LhsType>::Flags & RowMajorBit) ==
       (internal::traits<RhsType>::Flags & RowMajorBit))),
      THE_STORAGE_ORDER_OF_BOTH_SIDES_MUST_MATCH)

  using Lhs = typename internal::remove_all<LhsType>::type;
  using Rhs = typename internal::remove_all<RhsType>::type;
  using Base = typename internal::generic_xpr_base<
    HorizontalStack<LhsType, RhsType>>::type;

  EIGEN_GENERIC_PUBLIC_INTERFACE(HorizontalStack)

  HorizontalStack(const LhsType& lhs, const RhsType& rhs)
  : m_lhs(lhs), m_rhs(rhs)
  {
    eigen_assert(lhs.rows() == rhs.rows());
  }

  constexpr Index rows() const noexcept
  {
    constexpr auto LhsRowsAtCompileTime =
        internal::traits<LhsType>::RowsAtCompileTime;
    return LhsRowsAtCompileTime == Dynamic ? m_rhs.rows() : m_lhs.rows();
  }

  constexpr Index cols() const noexcept
  {
    return m_lhs.cols() + m_rhs.cols();
  }

  using LhsTypeNested = typename internal::ref_selector<LhsType>::type;
  using RhsTypeNested = typename internal::ref_selector<RhsType>::type;
  using LhsTypeNestedNoRef =
      typename internal::remove_reference<LhsTypeNested>::type;
  using RhsTypeNestedNoRef =
      typename internal::remove_reference<RhsTypeNested>::type;

  const LhsTypeNestedNoRef& lhs() const { return m_lhs; }

  const RhsTypeNestedNoRef& rhs() const { return m_rhs; }

 protected:
  LhsTypeNested m_lhs;
  RhsTypeNested m_rhs;
};


/**
 * Expression of a row by row concatenation (i.e. vertical stacking)
 * of two matrix (or array) expressions.
 *
 * Only sparse expressions are supported.
 */
template<typename LhsType, typename RhsType>
class VerticalStack : public internal::generic_xpr_base<
  VerticalStack<LhsType, RhsType>>::type, internal::no_assignment_operator
{
 public:
  EIGEN_STATIC_ASSERT_SAME_XPR_KIND(LhsType, RhsType)
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(LhsType, RhsType)
  EIGEN_STATIC_ASSERT((
      ((internal::traits<LhsType>::Flags & RowMajorBit) ==
       (internal::traits<RhsType>::Flags & RowMajorBit))),
      THE_STORAGE_ORDER_OF_BOTH_SIDES_MUST_MATCH)

  using Lhs = typename internal::remove_all<LhsType>::type;
  using Rhs = typename internal::remove_all<RhsType>::type;
  using Base = typename internal::generic_xpr_base<
    VerticalStack<LhsType, RhsType>>::type;

  EIGEN_GENERIC_PUBLIC_INTERFACE(VerticalStack)

  VerticalStack(const LhsType& lhs, const RhsType& rhs)
  : m_lhs(lhs), m_rhs(rhs)
  {
    eigen_assert(lhs.cols() == rhs.cols());
  }

  constexpr Index rows() const noexcept
  {
    return m_lhs.rows() + m_rhs.rows();
  }

  constexpr Index cols() const noexcept
  {
    constexpr auto LhsColsAtCompileTime =
        internal::traits<LhsType>::ColsAtCompileTime;
    return LhsColsAtCompileTime == Dynamic ? m_rhs.cols() : m_lhs.cols();
  }

  using LhsTypeNested = typename internal::ref_selector<LhsType>::type;
  using RhsTypeNested = typename internal::ref_selector<RhsType>::type;
  using LhsTypeNestedNoRef =
      typename internal::remove_reference<LhsTypeNested>::type;
  using RhsTypeNestedNoRef =
      typename internal::remove_reference<RhsTypeNested>::type;

  const LhsTypeNestedNoRef& lhs() const { return m_lhs; }

  const RhsTypeNestedNoRef& rhs() const { return m_rhs; }

 protected:
  LhsTypeNested m_lhs;
  RhsTypeNested m_rhs;
};

/**
 * Expression of a non-sequential sub-matrix defined by arbitrary sequences
 * of row and column indices.
 *
 * Only sparse expressions are supported.
 */
// NOTE(hidmic): this a *much* simplified equivalent to IndexedView in Eigen 3.4
template<typename XprType, typename RowIndices, typename ColIndices>
class View : public internal::generic_xpr_base<
  View<XprType, RowIndices, ColIndices>>::type, internal::no_assignment_operator
{
 public:
  using Base = typename internal::generic_xpr_base<
    View<XprType, RowIndices, ColIndices>>::type;
  using NestedExpression = typename internal::remove_all<XprType>::type;

  EIGEN_GENERIC_PUBLIC_INTERFACE(View)

  View(XprType& xpr, const RowIndices& rowIndices, const ColIndices& colIndices)
  : m_xpr(xpr), m_rowIndices(rowIndices), m_colIndices(colIndices)
  {
  }

  constexpr Index rows() const noexcept { return m_rowIndices.size(); }

  constexpr Index cols() const noexcept { return m_colIndices.size(); }

  const typename internal::remove_all<XprType>::type&
  nestedExpression() const { return m_xpr; }

  typename internal::remove_reference<XprType>::type&
  nestedExpression() { return m_xpr; }

  const RowIndices& rowIndices() const { return m_rowIndices; }

  const ColIndices& colIndices() const { return m_colIndices; }

 protected:
  using XprTypeNested =
      typename internal::ref_selector<XprType>::non_const_type;
  XprTypeNested m_xpr;
  RowIndices m_rowIndices;
  ColIndices m_colIndices;
};

namespace internal {

template<typename LhsType, typename RhsType>
struct evaluator<HorizontalStack<LhsType, RhsType>>
    : public binary_evaluator<HorizontalStack<LhsType, RhsType>>
{
  using XprType = HorizontalStack<LhsType, RhsType>;
  using Base = binary_evaluator<HorizontalStack<LhsType, RhsType>>;

  explicit evaluator(const XprType& xpr) : Base(xpr) {}
};

template<typename LhsType, typename RhsType>
struct binary_evaluator<
  HorizontalStack<LhsType, RhsType>, IteratorBased, IteratorBased
> : public evaluator_base<HorizontalStack<LhsType, RhsType>>
{
  using XprType = HorizontalStack<LhsType, RhsType>;
  using LhsIteratorType = typename evaluator<LhsType>::InnerIterator;
  using RhsIteratorType = typename evaluator<RhsType>::InnerIterator;
  using StorageIndex = typename traits<XprType>::StorageIndex;
  using Scalar = typename traits<XprType>::Scalar;

 public:
  class InnerIterator
  {
    enum {
      IsRowMajor = int(traits<XprType>::Flags) & RowMajorBit
    };

   public:
    InnerIterator(const binary_evaluator& eval, Index outer)
      : m_useLhsIter(IsRowMajor || outer < eval.m_lhsCols),
        m_lhsIter(eval.m_lhsEval, m_useLhsIter ? outer : 0),
        m_useRhsIter(IsRowMajor || outer >= eval.m_lhsCols),
        m_rhsIter(eval.m_rhsEval, m_useRhsIter ? (
            ! IsRowMajor ? outer - eval.m_lhsCols : outer) : 0),
        m_rhsOffset(IsRowMajor ? eval.m_lhsCols : 0),
        m_outer(outer)
    {
      this->operator++();
    }

    InnerIterator& operator++()
    {
      if (m_useLhsIter && m_lhsIter) {
        m_value = m_lhsIter.value();
        m_inner = m_lhsIter.index();
        ++m_lhsIter;
      } else if (m_useRhsIter && m_rhsIter) {
        m_value = m_rhsIter.value();
        m_inner = m_rhsOffset + m_rhsIter.index();
        ++m_rhsIter;
      } else {
        m_value = Scalar(0);
        m_inner = -1;
      }
      return *this;
    }

    Scalar value() const { return m_value; }
    Index index() const { return m_inner; }
    Index row() const { return IsRowMajor ? m_outer : index(); }
    Index col() const { return IsRowMajor ? index() : m_outer; }

    operator bool() const { return m_inner >= 0; }

   protected:
    bool m_useLhsIter;
    LhsIteratorType m_lhsIter;
    bool m_useRhsIter;
    RhsIteratorType m_rhsIter;
    StorageIndex m_rhsOffset;

    StorageIndex m_outer;
    StorageIndex m_inner;
    Scalar m_value;
  };

  enum {
    CoeffReadCost = (int(evaluator<LhsType>::CoeffReadCost) +
                     int(evaluator<RhsType>::CoeffReadCost)),
    Flags = int(evaluator<LhsType>::Flags) & RowMajorBit,
  };

  explicit binary_evaluator(const XprType& xpr)
    : m_lhsEval(xpr.lhs()),
      m_rhsEval(xpr.rhs()),
      m_lhsCols(xpr.lhs().cols())
  {
    EIGEN_INTERNAL_CHECK_COST_VALUE(CoeffReadCost);
  }

 protected:
  evaluator<LhsType> m_lhsEval;
  evaluator<RhsType> m_rhsEval;
  StorageIndex m_lhsCols;
};

template<typename LhsType, typename RhsType>
struct evaluator<VerticalStack<LhsType, RhsType>>
    : public binary_evaluator<VerticalStack<LhsType, RhsType>>
{
  using XprType = VerticalStack<LhsType, RhsType>;
  using Base = binary_evaluator<VerticalStack<LhsType, RhsType>>;

  explicit evaluator(const XprType& xpr) : Base(xpr) {}
};

template<typename LhsType, typename RhsType>
struct binary_evaluator<
  VerticalStack<LhsType, RhsType>, IteratorBased, IteratorBased
> : public evaluator_base<VerticalStack<LhsType, RhsType>>
{
  using XprType = VerticalStack<LhsType, RhsType>;
  using LhsIteratorType = typename evaluator<LhsType>::InnerIterator;
  using RhsIteratorType = typename evaluator<RhsType>::InnerIterator;
  using StorageIndex = typename traits<XprType>::StorageIndex;
  using Scalar = typename traits<XprType>::Scalar;

 public:
  class InnerIterator
  {
    enum {
      IsRowMajor = int(traits<XprType>::Flags) & RowMajorBit
    };

   public:
    InnerIterator(const binary_evaluator& eval, Index outer)
      : m_useLhsIter(!IsRowMajor || outer < eval.m_lhsRows),
        m_lhsIter(eval.m_lhsEval, m_useLhsIter ? outer : 0),
        m_useRhsIter(!IsRowMajor || outer >= eval.m_lhsRows),
        m_rhsIter(eval.m_rhsEval, m_useRhsIter ? (
            IsRowMajor ? outer - eval.m_lhsRows : outer) : 0),
        m_rhsOffset(!IsRowMajor ? eval.m_lhsRows : 0),
        m_outer(outer)
    {
      this->operator++();
    }

    InnerIterator& operator++()
    {
      if (m_useLhsIter && m_lhsIter) {
        m_value = m_lhsIter.value();
        m_inner = m_lhsIter.index();
        ++m_lhsIter;
      } else if (m_useRhsIter && m_rhsIter) {
        m_value = m_rhsIter.value();
        m_inner = m_rhsOffset + m_rhsIter.index();
        ++m_rhsIter;
      } else {
        m_value = Scalar(0);
        m_inner = -1;
      }
      return *this;
    }

    Scalar value() const { return m_value; }
    Index index() const { return m_inner; }
    Index row() const { return IsRowMajor ? m_outer : index(); }
    Index col() const { return IsRowMajor ? index() : m_outer; }

    operator bool() const { return m_inner >= 0; }

   protected:
    bool m_useLhsIter;
    LhsIteratorType m_lhsIter;
    bool m_useRhsIter;
    RhsIteratorType m_rhsIter;
    StorageIndex m_rhsOffset;

    StorageIndex m_outer;
    StorageIndex m_inner;
    Scalar m_value;
  };

  enum {
    CoeffReadCost = (int(evaluator<LhsType>::CoeffReadCost) +
                     int(evaluator<RhsType>::CoeffReadCost)),
    Flags = int(evaluator<LhsType>::Flags) & RowMajorBit,
  };

  explicit binary_evaluator(const XprType& xpr)
    : m_lhsEval(xpr.lhs()),
      m_rhsEval(xpr.rhs()),
      m_lhsRows(xpr.lhs().rows())
  {
    EIGEN_INTERNAL_CHECK_COST_VALUE(CoeffReadCost);
  }

 protected:
  evaluator<LhsType> m_lhsEval;
  evaluator<RhsType> m_rhsEval;
  StorageIndex m_lhsRows;
};

template<typename ArgType, typename RowIndices, typename ColIndices>
struct unary_evaluator<View<ArgType, RowIndices, ColIndices>, IteratorBased>
  : evaluator_base<View<ArgType, RowIndices, ColIndices>>
{
  using XprType = View<ArgType, RowIndices, ColIndices>;

  class InnerIterator
  {
    enum {
      IsRowMajor = traits<XprType>::IsRowMajor
    };
   public:
    using Scalar = typename traits<XprType>::Scalar;
    using StorageIndex = typename traits<XprType>::StorageIndex;

    InnerIterator(const unary_evaluator& eval, const Index outer)
    {
      const auto & outerIndices = constexpr_conditional<IsRowMajor>(
          eval.m_xpr.rowIndices(), eval.m_xpr.colIndices());
      const auto & innerIndices = constexpr_conditional<IsRowMajor>(
          eval.m_xpr.colIndices(), eval.m_xpr.rowIndices());
      using ArgIteratorType = typename evaluator<ArgType>::InnerIterator;
      for (ArgIteratorType it(eval.m_argImpl, outerIndices[outer]); it; ++it) {
        auto found = std::find(innerIndices.begin(), innerIndices.end(), it.index());
        if (found == innerIndices.end()) { continue; }
        const StorageIndex inner = std::distance(innerIndices.begin(), found);
        const StorageIndex row = IsRowMajor ? outer : inner;
        const StorageIndex col = IsRowMajor ? inner : outer;
        m_triplets.emplace_back(row, col, it.value());
      }
      std::sort(
          m_triplets.begin(), m_triplets.end(),
          [](const TripletType& a, const TripletType& b) {
            return IsRowMajor ? a.col() < b.col() : a.row() < b.row();
          });
      m_tripletsIter = m_triplets.begin();
    }

    InnerIterator& operator++()
    {
      ++m_tripletsIter;
      return *this;
    }

    Scalar value() const { return m_tripletsIter->value(); }
    StorageIndex index() const { return IsRowMajor? col() : row(); }
    StorageIndex row() const { return m_tripletsIter->row(); }
    StorageIndex col() const { return m_tripletsIter->col(); }

    operator bool() const { return m_tripletsIter != m_triplets.end(); }

   protected:
    using TripletType = Triplet<Scalar, StorageIndex>;
    std::vector<TripletType> m_triplets;
    typename std::vector<TripletType>::iterator m_tripletsIter;
  };

  enum {
    CoeffReadCost = evaluator<ArgType>::CoeffReadCost,

    FlagsRowMajorBit = traits<XprType>::FlagsRowMajorBit,

    Flags = evaluator<ArgType>::Flags & RowMajorBit,
  };

  explicit unary_evaluator(const XprType& xpr)
      : m_argImpl(xpr.nestedExpression()), m_xpr(xpr)
  {
    EIGEN_INTERNAL_CHECK_COST_VALUE(CoeffReadCost);
  }

protected:
  evaluator<ArgType> m_argImpl;
  const XprType& m_xpr;
};

}  // namespace internal
}  // namespace Eigen

namespace karto
{
namespace contrib
{

/**
 * Stacks two matrix expressions horizontally
 * i.e. from left to right, column by column.
 */
template <typename LhsType, typename RhsType>
Eigen::HorizontalStack<LhsType, RhsType>
StackHorizontally(const LhsType& lhs, const RhsType& rhs)
{
  return Eigen::HorizontalStack<LhsType, RhsType>(lhs, rhs);
}

/**
 * Stacks two matrix expressions vertically
 * i.e. from left to right, row by row.
 */
template <typename LhsType, typename RhsType>
Eigen::VerticalStack<LhsType, RhsType>
StackVertically(const LhsType& lhs, const RhsType& rhs)
{
  return Eigen::VerticalStack<LhsType, RhsType>(lhs, rhs);
}

/**
 * Arranges a view of a matrix expression defined by
 * arbitrary sequences of row and column indices.
 */
template<
  typename XprType,
  typename RowIndices = std::vector<
    typename Eigen::internal::traits<XprType>::StorageIndex>,
  typename ColIndices = std::vector<
    typename Eigen::internal::traits<XprType>::StorageIndex>>
Eigen::View<XprType, RowIndices, ColIndices>
ArrangeView(XprType & xpr, const RowIndices & rowIndices, const ColIndices & colIndices)
{
  return Eigen::View<XprType, RowIndices, ColIndices>(xpr, rowIndices, colIndices);
}

/** Computes the Moore-Penrose pseudoinverse of a dense matrix. */
template <typename Derived>
auto ComputeGeneralizedInverse(const Eigen::MatrixBase<Derived>& matrix)
{
  auto svd = matrix.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto values = svd.singularValues();
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (values(i) > 0.) { values(i) = 1. / values(i); }
  }
  auto sigma_generalized_inverse = Eigen::DiagonalWrapper{values};
  auto matrix_generalized_inverse =
      svd.matrixV() * sigma_generalized_inverse * svd.matrixU().transpose();
  return matrix_generalized_inverse.eval();
}

/** Computes the Moore-Penrose pseudoinverse of a sparse matrix. */
template <typename Derived>
auto ComputeGeneralizedInverse(const Eigen::SparseMatrixBase<Derived>& matrix)
{
  return ComputeGeneralizedInverse(matrix.toDense());
}


}  // namespace contrib
}  // namespace karto

#endif // KARTO_SDK__EIGEN_EXTENSIONS_H_
