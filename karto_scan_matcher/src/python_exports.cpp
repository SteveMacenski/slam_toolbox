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
 *
 */

/**
 * \file 
 * 
 * Boost Python exports for Karto scan matching
 *
 * \author Bhaskara Marthi
 */

#include <karto_scan_matcher/karto_scan_matcher.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace karto_scan_matcher
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;


BOOST_PYTHON_MODULE(karto_scan_matching)
{
  using namespace boost::python;
  using sm::LaserScan;
  using gm::Pose2D;
  using boost::shared_ptr;
  using std_msgs::Header;
  using ros::Time;

  class_<LaserScan, shared_ptr<LaserScan> >("LaserScanCpp", init<>())
    .def_readwrite("header", &LaserScan::header)
    .def_readwrite("angle_min", &LaserScan::angle_min)
    .def_readwrite("angle_max", &LaserScan::angle_max)
    .def_readwrite("angle_increment", &LaserScan::angle_increment)
    .def_readwrite("range_min", &LaserScan::range_min)
    .def_readwrite("range_max", &LaserScan::range_max)
    .def_readwrite("ranges", &LaserScan::ranges)
    ;

  class_<Pose2D>("Pose2DCpp", init<>())
    .def_readwrite("x", &Pose2D::x)
    .def_readwrite("y", &Pose2D::y)
    .def_readwrite("theta", &Pose2D::theta)
    ;

  class_<ScanMatchResult>("ScanMatchResultCpp", init<>())
    .def_readonly("pose", &ScanMatchResult::pose)
    .def_readonly("response", &ScanMatchResult::response)
    ;

  class_<ScanWithPose>("ScanWithPoseCpp", init<LaserScan, Pose2D>());

  class_<KartoScanMatcher>("KartoScanMatcherCpp",
                           init<LaserScan, Pose2D, double, double>())
    .def("scan_match", &KartoScanMatcher::scanMatch)
    ;

  class_<Header>("Header", init<>())
    .def_readwrite("seq", &Header::seq)
    .def_readwrite("stamp", &Header::stamp)
    .def_readwrite("frame_id", &Header::frame_id)
    ;

  class_<Time>("Time", init<>())
    .def_readwrite("sec", &Time::sec)
    .def_readwrite("nsec", &Time::nsec)
    ;


}

} // namespace
