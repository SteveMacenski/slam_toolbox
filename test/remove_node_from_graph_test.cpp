// Copyright 2026 spicysharkco
// SPDX-License-Identifier: LGPL-2.1-only

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "karto_sdk/Mapper.h"

namespace
{

// Solver that ignores everything, so the test only exercises the graph bookkeeping.
class NullSolver : public karto::ScanSolver
{
public:
  void Compute() override {}
  void Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr) override {}
  const IdPoseVector & GetCorrections() const override {return corrections_;}

private:
  IdPoseVector corrections_;
};

using Vertex = karto::Vertex<karto::LocalizedRangeScan>;

const double kRoomWidth = 10.0;
const double kRoomHeight = 8.0;

// Ranges seen from (x, y) inside an empty rectangular room.
std::vector<kt_double> roomRanges(const karto::LaserRangeFinder & laser, double x, double y)
{
  std::vector<kt_double> ranges;
  for (unsigned int i = 0; i < laser.GetNumberOfRangeReadings(); ++i) {
    const double angle = laser.GetMinimumAngle() + i * laser.GetAngularResolution();
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    double range = laser.GetRangeThreshold();
    if (dx > 1e-9) {range = std::min(range, (kRoomWidth - x) / dx);}
    if (dx < -1e-9) {range = std::min(range, -x / dx);}
    if (dy > 1e-9) {range = std::min(range, (kRoomHeight - y) / dy);}
    if (dy < -1e-9) {range = std::min(range, -y / dy);}
    ranges.push_back(range);
  }
  return ranges;
}

class RemoveNodeFromGraphTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    laser_ = karto::LaserRangeFinder::CreateLaserRangeFinder(
      karto::LaserRangeFinder_Custom, karto::Name("remove_node_test_laser"));
    laser_->SetMinimumRange(0.0);
    laser_->SetMaximumRange(30.0);
    laser_->SetMinimumAngle(-M_PI);
    laser_->SetMaximumAngle(M_PI - M_PI / 180.0);
    laser_->SetAngularResolution(M_PI / 180.0);
    laser_->SetIs360Laser(true);
    laser_->SetRangeThreshold(20.0);
    karto::SensorManager::GetInstance()->RegisterSensor(laser_, true);

    mapper_ = std::make_unique<karto::Mapper>();
    mapper_->setParamDoLoopClosing(false);
    mapper_->SetScanSolver(&solver_);
    mapper_->Initialize(laser_->GetRangeThreshold());

    for (int i = 0; i < 8; ++i) {
      auto * scan = new karto::LocalizedRangeScan(
        laser_->GetName(), roomRanges(*laser_, 2.0 + 0.6 * i, 4.0));
      const karto::Pose2 pose(2.0 + 0.6 * i, 4.0, 0.0);
      scan->SetOdometricPose(pose);
      scan->SetCorrectedPose(pose);
      scan->SetTime(static_cast<double>(i));
      ASSERT_TRUE(mapper_->Process(scan));
      scans_.push_back(scan);
    }
  }

  void TearDown() override
  {
    mapper_.reset();
    karto::SensorManager::GetInstance()->UnregisterSensor(laser_);
    delete laser_;
  }

  Vertex * vertexOf(const karto::LocalizedRangeScan * scan) const
  {
    return mapper_->GetGraph()->GetVertices().at(scan->GetSensorName()).at(scan->GetStateId());
  }

  size_t vertexCount() const
  {
    size_t count = 0;
    for (const auto & sensor : mapper_->GetGraph()->GetVertices()) {
      count += sensor.second.size();
    }
    return count;
  }

  static bool touches(const karto::Edge<karto::LocalizedRangeScan> * edge, const Vertex * vertex)
  {
    return edge->GetSource() == vertex || edge->GetTarget() == vertex;
  }

  NullSolver solver_;
  karto::LaserRangeFinder * laser_ = nullptr;
  std::unique_ptr<karto::Mapper> mapper_;
  std::vector<karto::LocalizedRangeScan *> scans_;
};

TEST_F(RemoveNodeFromGraphTest, RemovesVertexAndAllItsEdges)
{
  Vertex * removed = vertexOf(scans_[3]);
  const auto & graph_edges = mapper_->GetGraph()->GetEdges();

  size_t touching = 0;
  for (const auto * edge : graph_edges) {
    if (touches(edge, removed)) {++touching;}
  }
  ASSERT_GT(touching, 0u);
  const size_t edges_before = graph_edges.size();
  const size_t vertices_before = vertexCount();
  const std::vector<Vertex *> neighbours = removed->GetAdjacentVertices();
  ASSERT_FALSE(neighbours.empty());

  EXPECT_TRUE(mapper_->RemoveNodeFromGraph(removed));

  EXPECT_EQ(graph_edges.size(), edges_before - touching);
  EXPECT_EQ(vertexCount(), vertices_before - 1);
  for (const auto * edge : graph_edges) {
    EXPECT_FALSE(touches(edge, removed));
  }
  for (Vertex * neighbour : neighbours) {
    for (const auto * edge : neighbour->GetEdges()) {
      EXPECT_FALSE(touches(edge, removed));
    }
  }
  const auto & sensor_vertices = mapper_->GetGraph()->GetVertices().at(laser_->GetName());
  EXPECT_EQ(sensor_vertices.count(scans_[3]->GetStateId()), 0u);
}

TEST_F(RemoveNodeFromGraphTest, KeepsUnrelatedEdges)
{
  Vertex * removed = vertexOf(scans_[3]);
  Vertex * far_vertex = vertexOf(scans_[7]);
  const std::vector<Vertex *> neighbours = removed->GetAdjacentVertices();
  ASSERT_EQ(std::count(neighbours.begin(), neighbours.end(), far_vertex), 0);
  const size_t far_edges_before = far_vertex->GetEdges().size();
  const auto & graph_edges = mapper_->GetGraph()->GetEdges();

  std::vector<const karto::Edge<karto::LocalizedRangeScan> *> untouched;
  for (const auto * edge : graph_edges) {
    if (!touches(edge, removed)) {untouched.push_back(edge);}
  }
  ASSERT_FALSE(untouched.empty());

  ASSERT_TRUE(mapper_->RemoveNodeFromGraph(removed));

  // the surviving edges are the same objects, in the same order
  ASSERT_EQ(graph_edges.size(), untouched.size());
  for (size_t i = 0; i < untouched.size(); ++i) {
    EXPECT_EQ(graph_edges[i], untouched[i]);
  }
  EXPECT_EQ(far_vertex->GetEdges().size(), far_edges_before);
}

}  // namespace
