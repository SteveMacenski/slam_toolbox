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

#ifndef SLAM_TOOLBOX_TOOLBOX_MSGS_H_
#define SLAM_TOOLBOX_TOOLBOX_MSGS_H_

#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h> 
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "slam_toolbox_msgs/Pause.h"
#include "slam_toolbox_msgs/ClearQueue.h"
#include "slam_toolbox_msgs/ToggleInteractive.h"
#include "slam_toolbox_msgs/Clear.h"
#include "slam_toolbox_msgs/ClearQueue.h"
#include "slam_toolbox_msgs/SaveMap.h"
#include "slam_toolbox_msgs/LoopClosure.h"
#include "slam_toolbox_msgs/SerializePoseGraph.h"
#include "slam_toolbox_msgs/DeserializePoseGraph.h"
#include "slam_toolbox_msgs/MergeMaps.h"
#include "slam_toolbox_msgs/AddSubmap.h"

#endif //SLAM_TOOLBOX_TOOLBOX_MSGS_H_