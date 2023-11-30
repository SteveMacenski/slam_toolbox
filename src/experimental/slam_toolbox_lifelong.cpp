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

#include <algorithm>
#include <memory>
#include "slam_toolbox/experimental/slam_toolbox_lifelong.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
void LifelongSlamToolbox::checkIsNotNormalized(const double & value)
/*****************************************************************************/
{
  if (value < 0.0 || value > 1.0) {
    RCLCPP_FATAL(get_logger(),
      "All stores and scales must be in range [0, 1].");
    exit(-1);
  }
}

/*****************************************************************************/
LifelongSlamToolbox::LifelongSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
  RCLCPP_WARN(get_logger(), "Lifelong mapping mode in SLAM Toolbox is considered "
    "experimental and should be understood before proceeding. Please visit: "
    "https://github.com/SteveMacenski/slam_toolbox/wiki/"
    "Experimental-Lifelong-Mapping-Node for more information.");
}

/*****************************************************************************/
CallbackReturn
LifelongSlamToolbox::on_configure(const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  SlamToolbox::on_configure(state);

  use_tree_ = false;
  if (!this->has_parameter("lifelong_search_use_tree")) {
    this->declare_parameter("lifelong_search_use_tree", use_tree_);
  }
  use_tree_ = this->get_parameter("lifelong_search_use_tree").as_bool();

  iou_thresh_ = 0.10;
  if (!this->has_parameter("lifelong_minimum_score")) {
    this->declare_parameter("lifelong_minimum_score", iou_thresh_);
  }
  iou_thresh_ = this->get_parameter("lifelong_minimum_score").as_double();

  iou_match_ = 0.85;
  if (!this->has_parameter("lifelong_iou_match")) {
    this->declare_parameter("lifelong_iou_match", iou_match_);
  }
  iou_match_ = this->get_parameter("lifelong_iou_match").as_double();

  removal_score_ = 0.10;
  if (!this->has_parameter("lifelong_node_removal_score")) {
    this->declare_parameter("lifelong_node_removal_score", removal_score_);
  }
  removal_score_ = this->get_parameter("lifelong_node_removal_score").as_double();

  overlap_scale_ = 0.5;
  if (!this->has_parameter("lifelong_overlap_score_scale")) {
    this->declare_parameter("lifelong_overlap_score_scale", overlap_scale_);
  }
  overlap_scale_ = this->get_parameter("lifelong_overlap_score_scale").as_double();

  constraint_scale_ = 0.05;
  if (!this->has_parameter("lifelong_constraint_multiplier")) {
    this->declare_parameter("lifelong_constraint_multiplier", constraint_scale_);
  }
  constraint_scale_ = this->get_parameter("lifelong_constraint_multiplier").as_double();

  nearby_penalty_ = 0.001;
  if (!this->has_parameter("lifelong_nearby_penalty")) {
    this->declare_parameter("lifelong_nearby_penalty", nearby_penalty_);
  }
  nearby_penalty_ = this->get_parameter("lifelong_nearby_penalty").as_double();

  candidates_scale_ = 0.03;
  if (!this->has_parameter("lifelong_candidates_scale")) {
    this->declare_parameter("lifelong_candidates_scale", candidates_scale_);
  }
  candidates_scale_ = this->get_parameter("lifelong_candidates_scale").as_double();

  checkIsNotNormalized(iou_thresh_);
  checkIsNotNormalized(constraint_scale_);
  checkIsNotNormalized(removal_score_);
  checkIsNotNormalized(overlap_scale_);
  checkIsNotNormalized(iou_match_);
  checkIsNotNormalized(nearby_penalty_);
  checkIsNotNormalized(candidates_scale_);

  // in lifelong mode, we cannot have interactive mode enabled
  enable_interactive_mode_ = false;
  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
void LifelongSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(get_logger(), "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // LTS additional bounded node increase parameter (rate, or total for run or at all?)
  // LTS pseudo-localization mode. If want to add a scan, but
  // not deleting a scan, add to local buffer?
  // LTS if (eval() && dont_add_more_scans) {addScan()} else {localization_add_scan()}
  // LTS if (eval() && ctr / total < add_rate_scans) {addScan()} else {localization_add_scan()}
  LocalizedRangeScan * range_scan = addScan(laser, scan, pose);
  evaluateNodeDepreciation(range_scan);
}

/*****************************************************************************/
void LifelongSlamToolbox::evaluateNodeDepreciation(
  LocalizedRangeScan * range_scan)
/*****************************************************************************/
{
  if (range_scan) {
    boost::mutex::scoped_lock lock(smapper_mutex_);

    const BoundingBox2 & bb = range_scan->GetBoundingBox();
    const Size2<double> bb_size = bb.GetSize();
    double radius = sqrt(bb_size.GetWidth() * bb_size.GetWidth() +
        bb_size.GetHeight() * bb_size.GetHeight()) / 2.0;
    Vertices near_scan_vertices = FindScansWithinRadius(range_scan, radius);

    ScoredVertices scored_verices =
      computeScores(near_scan_vertices, range_scan);

    ScoredVertices::iterator it;
    for (it = scored_verices.begin(); it != scored_verices.end(); ++it) {
      if (it->GetScore() < removal_score_) {
        RCLCPP_DEBUG(get_logger(),
          "Removing node %i from graph with score: %f and old score: %f.",
          it->GetVertex()->GetObject()->GetUniqueId(),
          it->GetScore(), it->GetVertex()->GetScore());
        removeFromSlamGraph(it->GetVertex());
      } else {
        updateScoresSlamGraph(it->GetScore(), it->GetVertex());
      }
    }
  }
}

/*****************************************************************************/
Vertices LifelongSlamToolbox::FindScansWithinRadius(
  LocalizedRangeScan * scan, const double & radius)
/*****************************************************************************/
{
  // Using the tree will create a Kd-tree and find all neighbors in graph
  // around the reference scan. Otherwise it will use the graph and
  // access scans within radius that are connected with constraints to this
  // node.

  if (use_tree_) {
    return
      smapper_->getMapper()->GetGraph()->FindNearByVertices(
      scan->GetSensorName(), scan->GetBarycenterPose(), radius);
  } else {
    return
      smapper_->getMapper()->GetGraph()->FindNearLinkedVertices(scan, radius);
  }
}

/*****************************************************************************/
double LifelongSlamToolbox::computeObjectiveScore(
  const double & intersect_over_union,
  const double & area_overlap,
  const double & reading_overlap,
  const int & num_constraints,
  const double & initial_score,
  const int & num_candidates) const
/*****************************************************************************/
{
  // We have some useful metrics. lets make a new score
  // intersect_over_union: The higher this score, the better aligned in scope these scans are
  // area_overlap: The higher, the more area they share normalized by candidate size
  // reading_overlap: The higher, the more readings of the new scan the candidate contains
  // num_constraints: The lower, the less other nodes may rely on this candidate
  // initial_score: Last score of this vertex before update

  // this is a really good fit and not from a loop closure, lets just decay
  if (intersect_over_union > iou_match_ && num_constraints < 3) {
    return -1.0;
  }

  // to be conservative, lets say the overlap is the lesser of the
  // area and proportion of laser returns in the intersecting region.
  double overlap = overlap_scale_ * std::min(area_overlap, reading_overlap);

  // if the num_constraints are high we want to stave off the decay, but not override it
  double contraint_scale_factor = std::min(1.0,
      std::max(0., constraint_scale_ * (num_constraints - 2)));
  contraint_scale_factor = std::min(contraint_scale_factor, overlap);

  //
  double candidates = num_candidates - 1;
  double candidate_scale_factor = candidates_scale_ * candidates;

  // Give the initial score a boost proportional to the number of constraints
  // Subtract the overlap amount, apply a penalty for just being nearby
  // and scale the entire additional score by the number of candidates
  double score =
    initial_score * (1.0 + contraint_scale_factor) -
    overlap -
    nearby_penalty_;

  // score += (initial_score - score) * candidate_scale_factor;

  if (score > 1.0) {
    RCLCPP_ERROR(get_logger(),
      "Objective function calculated for vertex score (%0.4f)"
      " greater than one! Thresholding to 1.0", score);
    return 1.0;
  }

  return score;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeScore(
  LocalizedRangeScan * reference_scan,
  Vertex<LocalizedRangeScan> * candidate,
  const double & initial_score, const int & num_candidates)
/*****************************************************************************/
{
  double new_score = initial_score;
  LocalizedRangeScan * candidate_scan = candidate->GetObject();

  // compute metrics for information loss normalized
  double iou = computeIntersectOverUnion(reference_scan, candidate_scan);
  double area_overlap = computeAreaOverlapRatio(reference_scan, candidate_scan);
  int num_constraints = candidate->GetEdges().size();
  double reading_overlap = computeReadingOverlapRatio(reference_scan, candidate_scan);

  bool critical_lynchpoint = candidate_scan->GetUniqueId() == 0 ||
    candidate_scan->GetUniqueId() == 1;
  int id_diff = reference_scan->GetUniqueId() - candidate_scan->GetUniqueId();
  if (id_diff < smapper_->getMapper()->getParamScanBufferSize() ||
    critical_lynchpoint)
  {
    return initial_score;
  }

  double score = computeObjectiveScore(iou,
      area_overlap,
      reading_overlap,
      num_constraints,
      initial_score,
      num_candidates);

  RCLCPP_INFO(get_logger(), "Metric Scores: Initial: %f, IOU: %f,"
    " Area: %f, Num Con: %i, Reading: %f, outcome score: %f.",
    initial_score, iou, area_overlap, num_constraints, reading_overlap, score);
  return score;
}

/*****************************************************************************/
ScoredVertices LifelongSlamToolbox::computeScores(
  Vertices & near_scans,
  LocalizedRangeScan * range_scan)
/*****************************************************************************/
{
  ScoredVertices scored_vertices;
  scored_vertices.reserve(near_scans.size());

  // must have some minimum metric to utilize
  // IOU will drop sharply with fitment, I'd advise not setting this value
  // any higher than 0.15. Also check this is a linked constraint
  // We want to do this early to get a better estimate of local candidates
  ScanVector::iterator candidate_scan_it;
  double iou = 0.0;
  for (candidate_scan_it = near_scans.begin();
    candidate_scan_it != near_scans.end(); )
  {
    iou = computeIntersectOverUnion(range_scan,
        (*candidate_scan_it)->GetObject());
    if (iou < iou_thresh_ || (*candidate_scan_it)->GetEdges().size() < 2) {
      candidate_scan_it = near_scans.erase(candidate_scan_it);
    } else {
      ++candidate_scan_it;
    }
  }

  for (candidate_scan_it = near_scans.begin();
    candidate_scan_it != near_scans.end(); ++candidate_scan_it)
  {
    ScoredVertex scored_vertex((*candidate_scan_it),
      computeScore(range_scan, (*candidate_scan_it),
      (*candidate_scan_it)->GetScore(), near_scans.size()));
    scored_vertices.push_back(scored_vertex);
  }
  return scored_vertices;
}

/*****************************************************************************/
void LifelongSlamToolbox::removeFromSlamGraph(
  Vertex<LocalizedRangeScan> * vertex)
/*****************************************************************************/
{
  smapper_->getMapper()->RemoveNodeFromGraph(vertex);
  smapper_->getMapper()->GetMapperSensorManager()->RemoveScan(
    vertex->GetObject());
  dataset_->RemoveData(vertex->GetObject());
  vertex->RemoveObject();
  delete vertex;
  vertex = nullptr;
  // LTS what do we do about the contraints that node had about it?Nothing?Transfer?
}

/*****************************************************************************/
void LifelongSlamToolbox::updateScoresSlamGraph(
  const double & score,
  Vertex<LocalizedRangeScan> * vertex)
/*****************************************************************************/
{
  // Saved in graph so it persists between sessions and runs
  vertex->SetScore(score);
}

/*****************************************************************************/
bool LifelongSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_ERROR(get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

/*****************************************************************************/
void LifelongSlamToolbox::computeIntersectBounds(
  LocalizedRangeScan * s1, LocalizedRangeScan * s2,
  double & x_l, double & x_u, double & y_l, double & y_u)
/*****************************************************************************/
{
  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  Pose2 pose1 = s1->GetBarycenterPose();
  Pose2 pose2 = s2->GetBarycenterPose();

  const double s1_upper_x = pose1.GetX() + (bb1.GetWidth() / 2.0);
  const double s1_upper_y = pose1.GetY() + (bb1.GetHeight() / 2.0);
  const double s1_lower_x = pose1.GetX() - (bb1.GetWidth() / 2.0);
  const double s1_lower_y = pose1.GetY() - (bb1.GetHeight() / 2.0);

  const double s2_upper_x = pose2.GetX() + (bb2.GetWidth() / 2.0);
  const double s2_upper_y = pose2.GetY() + (bb2.GetHeight() / 2.0);
  const double s2_lower_x = pose2.GetX() - (bb2.GetWidth() / 2.0);
  const double s2_lower_y = pose2.GetY() - (bb2.GetHeight() / 2.0);

  x_u = std::min(s1_upper_x, s2_upper_x);
  y_u = std::min(s1_upper_y, s2_upper_y);
  x_l = std::max(s1_lower_x, s2_lower_x);
  y_l = std::max(s1_lower_y, s2_lower_y);
}

/*****************************************************************************/
double LifelongSlamToolbox::computeIntersect(
  LocalizedRangeScan * s1,
  LocalizedRangeScan * s2)
/*****************************************************************************/
{
  double x_l, x_u, y_l, y_u;
  computeIntersectBounds(s1, s2, x_l, x_u, y_l, y_u);
  const double intersect = (y_u - y_l) * (x_u - x_l);

  if (intersect < 0.0) {
    return 0.0;
  }

  return intersect;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeIntersectOverUnion(
  LocalizedRangeScan * s1,
  LocalizedRangeScan * s2)
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
  LocalizedRangeScan * ref_scan,
  LocalizedRangeScan * candidate_scan)
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
  LocalizedRangeScan * ref_scan,
  LocalizedRangeScan * candidate_scan)
/*****************************************************************************/
{
  const PointVectorDouble & pts = candidate_scan->GetPointReadings(true);
  const int num_pts = pts.size();

  // get the bounds of the intersect area
  double x_l, x_u, y_l, y_u;
  computeIntersectBounds(ref_scan, candidate_scan, x_l, x_u, y_l, y_u);

  PointVectorDouble::const_iterator pt_it;
  int inner_pts = 0;
  for (pt_it = pts.begin(); pt_it != pts.end(); ++pt_it) {
    if (pt_it->GetX() < x_u && pt_it->GetX() > x_l &&
      pt_it->GetY() < y_u && pt_it->GetY() > y_l)
    {
      inner_pts++;
    }
  }

  return static_cast<double>(inner_pts) / static_cast<double>(num_pts);
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::LifelongSlamToolbox)
