/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *
 * Unit tests for karto_scan_matcher
 * 
 * \author Bhaskara Marthi
 */


#include <karto_scan_matcher/karto_scan_matcher.h>
#include <gtest/gtest.h>
#include <boost/assign.hpp>

namespace ksm=karto_scan_matcher;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;

using boost::assign::operator+=;

const double PI=3.14159265;
const double TOL=1e-3;

bool approxEqual (const gm::Point& p1, const gm::Point& p2)
{
  return abs(p1.x-p2.x)+abs(p1.y-p2.y)+abs(p1.z-p2.z) < TOL;
}

TEST(KartoScanMatcher, Basic)
{
  gm::Pose2D laser;
  laser.x = 2.0;
  laser.y = 1.0;
  laser.theta = PI/4;
  
  sm::LaserScan scan;
  scan.angle_min = PI/4;
  scan.angle_max = 3*PI/4;
  scan.angle_increment = PI/4;
  scan.range_min = 0.0;
  scan.range_max = 5.0;
  scan.ranges += 1.0, 10.0, 2.0;

  ksm::ScanWithPose scan_with_pose(scan, laser);

  gm::Point barycenter;
  barycenter.x = 1.0;
  barycenter.y = 1.5;

  EXPECT_PRED2(approxEqual, barycenter, scan_with_pose.barycenter);
}

int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
