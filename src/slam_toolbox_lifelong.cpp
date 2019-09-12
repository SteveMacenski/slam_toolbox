/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2019, Samsung Research America
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
LifelongSlamToolbox::LifelongSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  loadPoseGraphByParams(nh);
  nh.param("lifelong_search_use_tree", use_tree_, false);
  nh.param("lifelong_minimum_score", iou_thresh_, 0.10);
  nh.param("lifelong_node_removal_score", removal_score_, 0.10);
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
double LifelongSlamToolbox::computeIntersect(LocalizedRangeScan* s1, 
  LocalizedRangeScan* s2) const
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

  const double x_u = std::min(s1_upper_x, s2_upper_x);
  const double y_u = std::min(s1_upper_y, s2_upper_y);
  const double x_l = std::max(s1_lower_x, s2_lower_x);
  const double y_l = std::max(s1_lower_y, s2_lower_y);

  const double intersect = (y_u - y_l) * (x_u - x_l);

  return intersect;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeIntersectOverUnion(LocalizedRangeScan* s1, 
  LocalizedRangeScan* s2) const
/*****************************************************************************/
{
  // this is a common metric in machine learning used to determine
  // the fitment of a set of bounding boxes. Its response sharply
  // drops by box matches.

  const double intersect = computeIntersect(s1, s2);

  if (intersect < 0.0)
  {
    return 0.0;
  }

  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  const double uni = (bb1.GetWidth() * bb1.GetHeight()) +
    (bb2.GetWidth() * bb2.GetHeight()) - intersect;

  return intersect / uni;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeOverlapRatio(LocalizedRangeScan* ref_scan, 
  LocalizedRangeScan* candidate_scan) const
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
  double overlap = computeOverlapRatio(reference_scan, candidate_scan);

  // TODO some equation: 
  //F(iou, overlap, % pts in ref scan overlap in candidate scan, scan match response, 
    //initial_score, tunable param, pose difference (position/orientation), 
    // bool for key frame (loop closure/lots of constraints)) = score

  return new_score;
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

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_toolbox");
  ros::NodeHandle nh("~");
  ros::spinOnce();

  int stack_size;
  if (nh.getParam("stack_size_to_use", stack_size))
  {
    ROS_INFO("Node using stack size %i", (int)stack_size);
    const rlim_t max_stack_size = stack_size;
    struct rlimit stack_limit;
    getrlimit(RLIMIT_STACK, &stack_limit);
    if (stack_limit.rlim_cur < stack_size)
    {
      stack_limit.rlim_cur = stack_size;
    }
    setrlimit(RLIMIT_STACK, &stack_limit);
  }

  slam_toolbox::LifelongSlamToolbox llst(nh);

  ros::spin();
  return 0;
}
