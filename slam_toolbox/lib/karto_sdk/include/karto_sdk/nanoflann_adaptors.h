/*
 * nanoflann_adaptors.h
 * Copyright (c) 2018, Simbe Robotics
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include "nanoflann.hpp"

// And this is the "dataset to kd-tree" adaptor class:
template <typename Derived>
struct VertexVectorPoseNanoFlannAdaptor
{
  const Derived &obj; //!< A const ref to the data set origin

  /// The constructor that sets the data set source
  VertexVectorPoseNanoFlannAdaptor(const Derived &obj_) : obj(obj_) { }

  /// CRTP helper method
  inline const Derived& derived() const { return obj; }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return derived().size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate value, the
  //  "if/else's" are actually solved at compile time.
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const
  {
    if (dim == 0) return derived()[idx]->GetObject()->GetCorrectedPose().GetX();
    else return derived()[idx]->GetObject()->GetCorrectedPose().GetY();
  }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

}; // end of VertexVectorPoseNanoFlannAdaptor

// And this is the "dataset to kd-tree" adaptor class:
template <typename Derived>
struct VertexVectorScanCenterNanoFlannAdaptor
{
  const Derived &obj;

  VertexVectorScanCenterNanoFlannAdaptor(const Derived &obj_) : obj(obj_) { }

  inline const Derived& derived() const { return obj; }

  inline size_t kdtree_get_point_count() const { return derived().size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const
  {
    if (dim == 0) return derived()[idx]->GetObject()->GetBarycenterPose().GetX();
    else return derived()[idx]->GetObject()->GetBarycenterPose().GetY();
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

}; // end of VertexVectorScanCenterNanoFlannAdaptor
