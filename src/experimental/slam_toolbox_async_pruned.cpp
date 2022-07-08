/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
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

#include "slam_toolbox/experimental/slam_toolbox_async_pruned.hpp"
#include <memory>

namespace slam_toolbox
{

/*****************************************************************************/
AsynchronousSlamToolboxPruned::AsynchronousSlamToolboxPruned(rclcpp::NodeOptions options)
  : SlamToolbox(options)
/*****************************************************************************/
{
  iou_threshold_ = this->declare_parameter("iou_threshold", iou_threshold_);
}

/*****************************************************************************/
void AsynchronousSlamToolboxPruned::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  //                                                                                      10 milliseconds
  if (!tf_->canTransform(odom_frame_, scan->header.frame_id, scan->header.stamp, rclcpp::Duration(10000000))) {
    RCLCPP_WARN(get_logger(), "Failed to get transform %s -> %s.", scan->header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }

  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder* laser = getLaser(scan);

  if (!laser)
  {
    RCLCPP_WARN(get_logger(),
                "Failed to create laser device for"
                " %s; discarding scan",
                scan->header.frame_id.c_str());
    return;
  }

  static int scan_ctr = 0;
  scan_ctr++;
  // throttled out
  if ((scan_ctr % throttle_scans_) != 0)
  {
    return;
  }

  LocalizedRangeScan* range_scan = addScan(laser, scan, pose);
  if (range_scan)
  {
    prune(range_scan);
  }
}

/*****************************************************************************/
void AsynchronousSlamToolboxPruned::prune(LocalizedRangeScan* range_scan)
/*****************************************************************************/
{
  RCLCPP_DEBUG(get_logger(), "pruning nearby nodes ...");
  boost::mutex::scoped_lock lock(smapper_mutex_);

  const BoundingBox2& bb      = range_scan->GetBoundingBox();
  const Size2<double> bb_size = bb.GetSize();
  double radius =
    sqrt(bb_size.GetWidth() * bb_size.GetWidth() + bb_size.GetHeight() * bb_size.GetHeight()) / 2.0;
  Vertices near_scan_vertices = smapper_->getMapper()->GetGraph()->FindNearByVertices(
    range_scan->GetSensorName(), range_scan->GetBarycenterPose(), radius);
  RCLCPP_DEBUG(get_logger(), "  found %zu nearby", near_scan_vertices.size());

  for (const auto& node : near_scan_vertices)
  {
    int id                   = node->GetObject()->GetUniqueId();
    bool critical_lynchpoint = id == 0 || id == 1;
    int id_diff              = range_scan->GetUniqueId() - id;
    if (id_diff < smapper_->getMapper()->getParamScanBufferSize() || critical_lynchpoint)
    {
      continue;
    }

    double iou = computeIntersectOverUnion(range_scan, node->GetObject());
    if (iou >= iou_threshold_)
    {
      RCLCPP_DEBUG(get_logger(), "  pruning <%d>, iou = %lf", id, iou);
      removeFromSlamGraph(node);
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "  keeping <%d>, iou = %lf", id, iou);
    }
  }
}

/*****************************************************************************/
void AsynchronousSlamToolboxPruned::removeFromSlamGraph(Vertex<LocalizedRangeScan>* vertex)
/*****************************************************************************/
{
  smapper_->getMapper()->RemoveNodeFromGraph(vertex);
  smapper_->getMapper()->GetMapperSensorManager()->RemoveScan(vertex->GetObject());
  dataset_->RemoveData(vertex->GetObject());
  vertex->RemoveObject();
  delete vertex;
  vertex = nullptr;
}

/*****************************************************************************/
bool AsynchronousSlamToolboxPruned::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE)
  {
    RCLCPP_WARN(get_logger(), "Requested a localization deserialization "
                              "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

/*****************************************************************************/
void AsynchronousSlamToolboxPruned::computeIntersectBounds(LocalizedRangeScan* s1,
                                                           LocalizedRangeScan* s2, double& x_l,
                                                           double& x_u, double& y_l, double& y_u)
/*****************************************************************************/
{
  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  Pose2 pose1       = s1->GetBarycenterPose();
  Pose2 pose2       = s2->GetBarycenterPose();

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
double AsynchronousSlamToolboxPruned::computeIntersect(LocalizedRangeScan* s1,
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
double AsynchronousSlamToolboxPruned::computeIntersectOverUnion(LocalizedRangeScan* s1,
                                                                LocalizedRangeScan* s2)
/*****************************************************************************/
{
  // this is a common metric in machine learning used to determine
  // the fitment of a set of bounding boxes. Its response sharply
  // drops by box matches.

  const double intersect = computeIntersect(s1, s2);

  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  const double uni =
    (bb1.GetWidth() * bb1.GetHeight()) + (bb2.GetWidth() * bb2.GetHeight()) - intersect;

  return intersect / uni;
}

} // namespace slam_toolbox
