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

#include "slam_toolbox/slam_toolbox_lifelong.hpp"

namespace slam_toolbox
{

// LTS the current_scans_ is a copy on the data stored by the dataset, can we hijack to not duplicate?
// LTS or add a parameter to disable interactive mode & not add to the scan holder/disable interactive mode cb
// LTS we keep increasing the vector of nodes/scans/constraints even though freeing the memory

/*****************************************************************************/
void LifelongSlamToolbox::checkIsNotNormalized(const double& value)
/*****************************************************************************/
{
  if (value < 0.0 || value > 1.0)
  {
    ROS_FATAL("All stores and scales must be in range [0, 1].");
    exit(-1);
  }
}

/*****************************************************************************/
LifelongSlamToolbox::LifelongSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  loadPoseGraphByParams(nh);
  nh.param("lifelong_search_use_tree", use_tree_, false);
  nh.param("lifelong_minimum_score", iou_thresh_, 0.10);
  nh.param("lifelong_iou_match", iou_match_, 0.75);
  nh.param("lifelong_node_removal_score", removal_score_, 0.10);
  nh.param("lifelong_overlap_score_scale", overlap_scale_, 0.5);
  nh.param("lifelong_constraint_multiplier", constraint_scale_, 0.05);
  nh.param("lifelong_nearby_penalty", nearby_penalty_, 0.001);

  checkIsNotNormalized(iou_thresh_);
  checkIsNotNormalized(constraint_scale_);
  checkIsNotNormalized(removal_score_);
  checkIsNotNormalized(overlap_scale_);
  checkIsNotNormalized(iou_match_);
}

/*****************************************************************************/
void LifelongSlamToolbox::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  // no odom info
  Pose2 pose;
  if(!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // LTS additional bounded node increase parameter (rate, or total for run or at all?)
  // LTS pseudo-localization mode. If want to add a scan, but not deleting a scan, add to local buffer?
  // LTS if (eval() && dont_add_more_scans) {addScan()} else {localization_add_scan()}
  // LTS if (eval() && ctr / total < add_rate_scans) {addScan()} else {localization_add_scan()}

  if (addScan(laser, scan, pose))
  {
    evaluateNodeDepreciation(getLocalizedRangeScan(laser, scan, pose));  
  }

  return;
}

/*****************************************************************************/
void LifelongSlamToolbox::evaluateNodeDepreciation(
  LocalizedRangeScan* range_scan)
/*****************************************************************************/
{
  ROS_FATAL("Eval node");
  if (range_scan)
  {
    boost::mutex::scoped_lock lock(smapper_mutex_);

    const BoundingBox2& bb = range_scan->GetBoundingBox();
    const Size2<double> bb_size = bb.GetSize();
    double radius = sqrt(bb_size.GetWidth()*bb_size.GetWidth() +
      bb_size.GetHeight()*bb_size.GetHeight()) / 2.0;
    Vertices near_scan_vertices = FindScansWithinRadius(range_scan, radius);

    ScoredVertices scored_verices =
      computeScores(near_scan_vertices, range_scan);

    ScoredVertices::iterator it;
    for (it = scored_verices.begin(); it != scored_verices.end(); ++it)
    {
      if (it->GetVertex()->GetScore() < removal_score_)
      {
        ROS_INFO("STEVE REMOVING NODE FROM GRAPH %f", it->GetVertex()->GetScore());
        removeFromSlamGraph(it->GetVertex());
      }
      else
      {
        updateScoresSlamGraph(it->GetVertex()->GetScore(), it->GetVertex());
      }
    }

    delete range_scan;
  }

  return;
}

/*****************************************************************************/
Vertices LifelongSlamToolbox::FindScansWithinRadius(
  LocalizedRangeScan* scan, const double& radius)
/*****************************************************************************/
{
  // Using the tree will create a Kd-tree and find all neighbors in graph
  // around the reference scan. Otherwise it will use the graph and
  // access scans within radius that are connected with constraints to this
  // node.

  if (use_tree_)
  {
    return
      smapper_->getMapper()->GetGraph()->FindNearByVertices(
      scan->GetSensorName(), scan->GetBarycenterPose(), radius);
  }
  else
  {
    return 
      smapper_->getMapper()->GetGraph()->FindNearLinkedVertices(scan, radius);
  }
}

/*****************************************************************************/
double LifelongSlamToolbox::computeObjectiveScore(
  const double& intersect_over_union,
  const double& area_overlap,
  const double& reading_overlap,
  const double& num_constraints,
  const double& initial_score) const
/*****************************************************************************/
{
  // LTS how to make sure sees enough unique area and not just the same 30% 3 times?

  // We have some useful metrics. lets make a new score
  // intersect_over_union: The higher this score, the better aligned in scope these scans are
  // area_overlap: The higher, the more area they share normalized by candidate size
  // reading_overlap: The higher, the more readings of the new scan the candidate contains
  // num_constraints: The lower, the less other nodes may rely on this candidate
  // initial_score: Last score of this vertex before update

  double score = 1.0;

  // this is a really good fit and not from a loop closure, lets just decay
  if (intersect_over_union > iou_match_ && num_constraints < 3)
  {
    return 0.0;
  }

  // to be conservative, lets say the overlap is the lesser of the
  // area and proportion of laser returns in the intersecting region.
  const double overlap = overlap_scale_ * std::min(area_overlap, reading_overlap);

  // if the num_constraints are high we want to stave off the decay, but not override it
  double contraint_scale_factor = std::min(1.0, std::max(0., constraint_scale_ * (num_constraints - 2)));
  contraint_scale_factor = std::min(contraint_scale_factor, overlap);

  const double unscaled_score = std::max(0., initial_score - overlap);
  std::cout << "unscaled score: " << unscaled_score << std::endl; 
  if (unscaled_score > 1.0)
  {
    ROS_ERROR("Objective function calculated for vertex score greater than "
      "one! Thresholding to 1.");
    return 1.0;
  }
  else if (unscaled_score < 0.0)
  {
    ROS_ERROR("Objective function calculated for vertex score less than "
      "zero! Thresholding to 0.");
    return 0.0;
  }
  else if (unscaled_score == 0.0)
  {
    return unscaled_score;
  }
  else
  {
    // Constraint help keep - new score - a small penalty for even being nearby
    score = std::min(1.0, unscaled_score * (1.0 + contraint_scale_factor) - nearby_penalty_);
    std::cout << "score: " << score << std::endl; 
    return score;


// metrics:
// IOU 0.359621
// area 1
// Num COn 3
// readings 0.951271
// unscaled score: 0.524364
// score: 0.549583
    // rising because we're multiplusing unscaled by num > 1, always more. Need to 
    // instead multiply on initial score 
  }
}

/*****************************************************************************/
double LifelongSlamToolbox::computeScore(
  LocalizedRangeScan* reference_scan,
  Vertex<LocalizedRangeScan>* candidate,
  const double& initial_score)
/*****************************************************************************/
{
  LocalizedRangeScan* candidate_scan = candidate->GetObject();
  double iou = computeIntersectOverUnion(reference_scan, candidate_scan);

  // must have some minimum metric to utilize
  // IOU will drop sharply with fitment, I'd advise not setting this value
  // any higher than 0.15.
  if (iou < iou_thresh_)
  {
    return initial_score;
  }

  double new_score = initial_score;

  // compute metric an information loss normalized metric
  double area_overlap = computeAreaOverlapRatio(reference_scan, candidate_scan);
  double num_constraints = candidate->GetEdges().size();
  double reading_overlap = computeReadingOverlapRatio(reference_scan, candidate_scan);

  std::cout << "metrics:" << std::endl;
  std::cout << "IOU " << iou << std::endl;
    std::cout << "area " << area_overlap << std::endl;
      std::cout << "Num COn " << num_constraints << std::endl;
        std::cout << "readings " << reading_overlap << std::endl;
  return computeObjectiveScore(iou,
                               area_overlap,
                               reading_overlap,
                               num_constraints,
                               initial_score);
}

/*****************************************************************************/
ScoredVertices
LifelongSlamToolbox::computeScores(
  Vertices& near_scans,
  LocalizedRangeScan* range_scan)
/*****************************************************************************/
{
  ScoredVertices scored_vertices;
  scored_vertices.reserve(near_scans.size());

  ScanVector::iterator candidate_scan_it;
  for (candidate_scan_it = near_scans.begin();
    candidate_scan_it != near_scans.end(); ++candidate_scan_it)
  {
    ScoredVertex scored_vertex(*candidate_scan_it,
      computeScore(range_scan, *candidate_scan_it,
        (*candidate_scan_it)->GetScore()));
    scored_vertices.push_back(scored_vertex);
  }
  return scored_vertices;
}

/*****************************************************************************/
void LifelongSlamToolbox::removeFromSlamGraph(
  Vertex<LocalizedRangeScan>* vertex)
/*****************************************************************************/
{
  dataset_->RemoveData(vertex->GetObject());
  smapper_->getMapper()->RemoveNodeFromGraph(vertex);
  smapper_->getMapper()->GetMapperSensorManager()->RemoveScan(
    vertex->GetObject());
  vertex->DeleteObject();
  delete vertex;
  vertex = nullptr;
  // LTS what do we do about the contraints that node had about it?Nothing?Transfer?
}

/*****************************************************************************/
void LifelongSlamToolbox::updateScoresSlamGraph(const double& score, 
  Vertex<LocalizedRangeScan>* vertex)
/*****************************************************************************/
{
  // Saved in graph so it persists between sessions and runs
  vertex->SetScore(score);
}

/*****************************************************************************/
bool LifelongSlamToolbox::deserializePoseGraphCallback(
  slam_toolbox::DeserializePoseGraph::Request& req,
  slam_toolbox::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
  if (req.match_type == procType::LOCALIZE_AT_POSE)
  {
    ROS_ERROR("Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

/*****************************************************************************/
void LifelongSlamToolbox::computeIntersectBounds(
  LocalizedRangeScan* s1, LocalizedRangeScan* s2,
  double& x_l, double& x_u, double& y_l, double& y_u)
/*****************************************************************************/
{
  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  Pose2 pose1 = s1->GetBarycenterPose();
  Pose2 pose2 = s2->GetBarycenterPose();

  const double s1_upper_x = pose1.GetX() + (bb1.GetWidth()  / 2.0);
  const double s1_upper_y = pose1.GetY() + (bb1.GetHeight() / 2.0);
  const double s1_lower_x = pose1.GetX() - (bb1.GetWidth()  / 2.0);
  const double s1_lower_y = pose1.GetY() - (bb1.GetHeight() / 2.0);

  const double s2_upper_x = pose2.GetX() + (bb2.GetWidth()  / 2.0);
  const double s2_upper_y = pose2.GetY() + (bb2.GetHeight() / 2.0);
  const double s2_lower_x = pose2.GetX() - (bb2.GetWidth()  / 2.0);
  const double s2_lower_y = pose2.GetY() - (bb2.GetHeight() / 2.0);

  x_u = std::min(s1_upper_x, s2_upper_x);
  y_u = std::min(s1_upper_y, s2_upper_y);
  x_l = std::max(s1_lower_x, s2_lower_x);
  y_l = std::max(s1_lower_y, s2_lower_y);
  return;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeIntersect(LocalizedRangeScan* s1, 
  LocalizedRangeScan* s2)
/*****************************************************************************/
{
  double x_l, x_u, y_l, y_u;
  computeIntersectBounds(s1, s2, x_l, x_u, y_l, y_u);
  const double intersect = (y_u - y_l) * (x_u - x_l);

  if (intersect < 0.0)
  {
    return 0.0;
  }

  return intersect;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeIntersectOverUnion(LocalizedRangeScan* s1, 
  LocalizedRangeScan* s2)
/*****************************************************************************/
{
  // this is a common metric in machine learning used to determine
  // the fitment of a set of bounding boxes. Its response sharply
  // drops by box matches.

  const double intersect = computeIntersect(s1, s2);

  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  const double uni = (bb1.GetWidth() * bb1.GetHeight()) +
    (bb2.GetWidth() * bb2.GetHeight()) - intersect;

  return intersect / uni;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeAreaOverlapRatio(
  LocalizedRangeScan* ref_scan, 
  LocalizedRangeScan* candidate_scan)
/*****************************************************************************/
{
  // ref scan is new scan, candidate scan is potential for decay
  // so we want to find the ratio of space of the candidate scan 
  // the reference scan takes up

  double overlap_area = computeIntersect(ref_scan, candidate_scan);
  Size2<double> bb_candidate = candidate_scan->GetBoundingBox().GetSize();
  const double candidate_area = 
    bb_candidate.GetHeight() * bb_candidate.GetWidth();

  return overlap_area / candidate_area;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeReadingOverlapRatio(
  LocalizedRangeScan* ref_scan, 
  LocalizedRangeScan* candidate_scan)
/*****************************************************************************/
{
  const PointVectorDouble& pts = candidate_scan->GetPointReadings(true);
  const int num_pts = pts.size();

  // get the bounds of the intersect area
  double x_l, x_u, y_l, y_u;
  computeIntersectBounds(ref_scan, candidate_scan, x_l, x_u, y_l, y_u);

  PointVectorDouble::const_iterator pt_it;
  int inner_pts = 0;
  for (pt_it = pts.begin(); pt_it != pts.end(); ++pt_it)
  {
    if (pt_it->GetX() < x_u && pt_it->GetX() > x_l &&
        pt_it->GetY() < y_u && pt_it->GetY() > y_l)
    {
      inner_pts++;
    }
  }

  return double(inner_pts) / double(num_pts);
}

} // end namespace
