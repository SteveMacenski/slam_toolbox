/*
 * slam_toolbox
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

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_LIFELONG_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_LIFELONG_H_

#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

using namespace ::karto;

class LifelongSlamToolbox : public SlamToolbox
{
public:
  LifelongSlamToolbox(ros::NodeHandle& nh);
  ~LifelongSlamToolbox() {};

protected:
  virtual void laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan) override final;
  virtual bool deserializePoseGraphCallback(
    slam_toolbox::DeserializePoseGraph::Request& req,
    slam_toolbox::DeserializePoseGraph::Response& resp) override final;

  void evaluateNodeDepreciation(LocalizedRangeScan* range_scan);
  void removeFromSlamGraph(Vertex<LocalizedRangeScan>* vertex);
  double computeScore(LocalizedRangeScan* reference_scan, Vertex<LocalizedRangeScan>* candidate, const double& initial_score);
  ScoredVertices computeScores(Vertices& near_scans, LocalizedRangeScan* range_scan);
  Vertices FindScansWithinRadius(LocalizedRangeScan* scan, const double& radius);
  void updateScoresSlamGraph(const double& score, Vertex<LocalizedRangeScan>* vertex);
  void checkIsNotNormalized(const double& value);

  // computation metrics
  double computeIntersect(LocalizedRangeScan* s1, LocalizedRangeScan* s2) const;
  double computeIntersectOverUnion(LocalizedRangeScan* s1, LocalizedRangeScan* s2) const;
  double computeAreaOverlapRatio(LocalizedRangeScan* ref_scan, LocalizedRangeScan* candidate_scan) const;
  double computeReadingOverlapRatio(LocalizedRangeScan* ref_scan, LocalizedRangeScan* candidate_scan) const;
  double computeObjectiveScore(const double& intersect_over_union, const double& area_overlap, const double& reading_overlap, const double& num_constraints, const double& initial_score);
  void computeIntersectBounds(LocalizedRangeScan* s1, LocalizedRangeScan* s2, double& x_l, double& x_u, double& y_l, double& y_u) const;

  bool use_tree_;
  double iou_thresh_;
  double removal_score_;
  double overlap_scale_;
  double constraint_scale_;
  double iou_match_;
  double nearby_penalty_;
};

}

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_LIFELONG_H_
