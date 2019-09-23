/*
 * slam_toolbox
 * Copyright (c) 2019, Steve Macenski
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

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_LIFELONG_TEST_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_LIFELONG_TEST_H_

#include <gtest/gtest.h>
#include "slam_toolbox/experimental/slam_toolbox_lifelong.hpp"

using namespace karto;

// 3 potential test cases, t1 is used.
//t1 = IOU([3.5, 4.0, 3.0, 4.0], [3.5, 5.5, 3.0, 3.0]) == 6.0
//t2 = IOU([4.5, 3.0, 5.0, 2.0], [4.5, 4.5, 3, 3]) == 3.0
//t3 = IOU([4.5, 3.5, 3.0, 3.0], [2.5, 5.5, 3, 3]) == 1.0

namespace 
{

TEST(LifelingMetricsTests, TestBounds)
{
  LocalizedRangeScan* s1 = new LocalizedRangeScan(); 
  LocalizedRangeScan* s2 = new LocalizedRangeScan(); 
  Pose2 p1 = Pose2(3.5, 4.0, 0.0);
  Pose2 p2 = Pose2(3.5, 5.5, 0.0);
  s1->SetBarycenterPose(p1);
  s2->SetBarycenterPose(p2);
  BoundingBox2 bb1, bb2;
  bb1.SetMinimum(Vector2<kt_double>(2.0, 2.0));
  bb1.SetMaximum(Vector2<kt_double>(5.0, 6.0));
  bb2.SetMinimum(Vector2<kt_double>(2.0, 4.0));
  bb2.SetMaximum(Vector2<kt_double>(5.0, 7.0));
  s1->SetBoundingBox(bb1);
  s2->SetBoundingBox(bb2);
  PointVectorDouble pts;
  pts.push_back(Vector2<double>(3.0, 5.0)); //inside
  pts.push_back(Vector2<double>(3.0, 3.0)); //outside
  s2->SetPointReadings(pts, true);
  double x_l, x_u, y_l, y_u;
  bool dirty = false;
  s1->SetIsDirty(dirty);
  s2->SetIsDirty(dirty);
  slam_toolbox::LifelongSlamToolbox::computeIntersectBounds(s1, s2, x_l, x_u, y_l, y_u);
  EXPECT_EQ(x_l, 2.0);
  EXPECT_EQ(x_u, 5.0);
  EXPECT_EQ(y_l, 4.0);
  EXPECT_EQ(y_u, 6.0);
  delete s1;
  delete s2;
}

TEST(LifelingMetricsTests, TestIntersect)
{
  LocalizedRangeScan* s1 = new LocalizedRangeScan(); 
  LocalizedRangeScan* s2 = new LocalizedRangeScan(); 
  Pose2 p1 = Pose2(3.5, 4.0, 0.0);
  Pose2 p2 = Pose2(3.5, 5.5, 0.0);
  s1->SetBarycenterPose(p1);
  s2->SetBarycenterPose(p2);
  BoundingBox2 bb1, bb2;
  bb1.SetMinimum(Vector2<kt_double>(2.0, 2.0));
  bb1.SetMaximum(Vector2<kt_double>(5.0, 6.0));
  bb2.SetMinimum(Vector2<kt_double>(2.0, 4.0));
  bb2.SetMaximum(Vector2<kt_double>(5.0, 7.0));
  s1->SetBoundingBox(bb1);
  s2->SetBoundingBox(bb2);
  PointVectorDouble pts;
  pts.push_back(Vector2<double>(3.0, 5.0)); //inside
  pts.push_back(Vector2<double>(3.0, 3.0)); //outside
  s2->SetPointReadings(pts, true);
  bool dirty = false;
  s1->SetIsDirty(dirty);
  s2->SetIsDirty(dirty);
  double intersect = slam_toolbox::LifelongSlamToolbox::computeIntersect(s1, s2);
  EXPECT_EQ(intersect, 6.0);
  delete s1;
  delete s2;
}

TEST(LifelingMetricsTests, TestIntersectOverUnion)
{
  LocalizedRangeScan* s1 = new LocalizedRangeScan(); 
  LocalizedRangeScan* s2 = new LocalizedRangeScan(); 
  Pose2 p1 = Pose2(3.5, 4.0, 0.0);
  Pose2 p2 = Pose2(3.5, 5.5, 0.0);
  s1->SetBarycenterPose(p1);
  s2->SetBarycenterPose(p2);
  BoundingBox2 bb1, bb2;
  bb1.SetMinimum(Vector2<kt_double>(2.0, 2.0));
  bb1.SetMaximum(Vector2<kt_double>(5.0, 6.0));
  bb2.SetMinimum(Vector2<kt_double>(2.0, 4.0));
  bb2.SetMaximum(Vector2<kt_double>(5.0, 7.0));
  s1->SetBoundingBox(bb1);
  s2->SetBoundingBox(bb2);
  PointVectorDouble pts;
  pts.push_back(Vector2<double>(3.0, 5.0)); //inside
  pts.push_back(Vector2<double>(3.0, 3.0)); //outside
  s2->SetPointReadings(pts, true);
  bool dirty = false;
  s1->SetIsDirty(dirty);
  s2->SetIsDirty(dirty);
  double intersect_over_union = slam_toolbox::LifelongSlamToolbox::computeIntersectOverUnion(s1, s2);
  EXPECT_EQ(intersect_over_union, 0.4);
  delete s1;
  delete s2;
}

TEST(LifelingMetricsTests, TestAreaOverlap)
{
  LocalizedRangeScan* s1 = new LocalizedRangeScan(); 
  LocalizedRangeScan* s2 = new LocalizedRangeScan(); 
  Pose2 p1 = Pose2(3.5, 4.0, 0.0);
  Pose2 p2 = Pose2(3.5, 5.5, 0.0);
  s1->SetBarycenterPose(p1);
  s2->SetBarycenterPose(p2);
  BoundingBox2 bb1, bb2;
  bb1.SetMinimum(Vector2<kt_double>(2.0, 2.0));
  bb1.SetMaximum(Vector2<kt_double>(5.0, 6.0));
  bb2.SetMinimum(Vector2<kt_double>(2.0, 4.0));
  bb2.SetMaximum(Vector2<kt_double>(5.0, 7.0));
  s1->SetBoundingBox(bb1);
  s2->SetBoundingBox(bb2);
  PointVectorDouble pts;
  pts.push_back(Vector2<double>(3.0, 5.0)); //inside
  pts.push_back(Vector2<double>(3.0, 3.0)); //outside
  s2->SetPointReadings(pts, true);
  bool dirty = false;
  s1->SetIsDirty(dirty);
  s2->SetIsDirty(dirty);
  double area = slam_toolbox::LifelongSlamToolbox::computeAreaOverlapRatio(s1, s2);
  EXPECT_NEAR(area, 0.6666, 0.01);
  delete s1;
  delete s2;
}

TEST(LifelingMetricsTests, TestPtOverlap)
{
  LocalizedRangeScan* s1 = new LocalizedRangeScan(); 
  LocalizedRangeScan* s2 = new LocalizedRangeScan(); 
  Pose2 p1 = Pose2(3.5, 4.0, 0.0);
  Pose2 p2 = Pose2(3.5, 5.5, 0.0);
  s1->SetBarycenterPose(p1);
  s2->SetBarycenterPose(p2);
  BoundingBox2 bb1, bb2;
  bb1.SetMinimum(Vector2<kt_double>(2.0, 2.0));
  bb1.SetMaximum(Vector2<kt_double>(5.0, 6.0));
  bb2.SetMinimum(Vector2<kt_double>(2.0, 4.0));
  bb2.SetMaximum(Vector2<kt_double>(5.0, 7.0));
  s1->SetBoundingBox(bb1);
  s2->SetBoundingBox(bb2);
  PointVectorDouble pts;
  pts.push_back(Vector2<double>(3.0, 5.0)); //inside
  pts.push_back(Vector2<double>(3.0, 3.0)); //outside
  s2->SetPointReadings(pts, true);
  bool dirty = false;
  s1->SetIsDirty(dirty);
  s2->SetIsDirty(dirty);
  double area = slam_toolbox::LifelongSlamToolbox::computeReadingOverlapRatio(s1, s2);
  EXPECT_EQ(area, 0.5);
  delete s1;
  delete s2;
}

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_LIFELONG_TEST_H_
