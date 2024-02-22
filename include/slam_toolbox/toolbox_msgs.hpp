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

#ifndef SLAM_TOOLBOX__TOOLBOX_MSGS_HPP_
#define SLAM_TOOLBOX__TOOLBOX_MSGS_HPP_

#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#include "slam_toolbox/srv/pause.hpp"
#include "slam_toolbox/srv/reset.hpp"
#include "slam_toolbox/srv/clear_queue.hpp"
#include "slam_toolbox/srv/toggle_interactive.hpp"
#include "slam_toolbox/srv/clear.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "slam_toolbox/srv/loop_closure.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "slam_toolbox/srv/deserialize_pose_graph.hpp"
#include "slam_toolbox/srv/merge_maps.hpp"
#include "slam_toolbox/srv/add_submap.hpp"

#endif  // SLAM_TOOLBOX__TOOLBOX_MSGS_HPP_
