import roslib
roslib.load_manifest('karto_scan_matcher')

import geometry_msgs.msg as gm
import sensor_msgs.msg as sm

from karto_scan_matching import *

def make_matcher(init_scan, laser_pose, search_size, search_resolution):
    """
    @param init_scan: Initial scan used only to read parameters of laser scanner
    @type init_scan: sensor_msgs.msg.LaserScan
    @param laser_pose: Pose of laser wrt base frame
    @type laser_pose: Tuple of form (x, y, theta)
    @param search_size: Radius of square to do matching in (meters)
    @param search_resolution: Resolution of grid search (meters)
    @return Object of type KartoScanMatcher, which can be used as an argument to match_scans.
    """
    scan = scan_from_ros(init_scan)
    pose = pose_from_tuple(p)
    return KartoScanMatcherCpp(scan, pose, search_size, search_resolution)


def match_scans(matcher, scan, init_pose, ref_scans):
    """
    @param matcher: Object returned by make_matcher
    @param scan: Scan to match
    @type scan: sensor_msgs.msg.LaserScan
    @param init_pose: Initial pose estimate
    @type init_pose: Tuple of form (x, y, theta)
    @param ref_scans: Reference scans
    @type ref_scans: Vector of tuples ((x, y, theta), sensor_msgs.msg.LaserScan)
    @return Tuple (opt_pose, response)
    """
    res = matcher.scanMatch(scan_from_ros(scan), pose_from_tuple(init_pose),
                            convert_ref_scans(ref_scans))
    return ((res.pose.x, res.pose.y, res.pose.theta), res.response)




############################################################
# Internal
############################################################

def scan_from_ros(s):
    scan = LaserScan()
    scan.header.frame_id = s.header.stamp.frame_id
    scan.range_min = s.range_min
    scan.range_max = s.range_max
    scan.angle_min = s.angle_min
    scan.angle_max = s.angle_max
    scan.angle_increment = s.angle_increment
    scan.ranges = s.ranges
    return scan
    
def pose_from_tuple(p):
    pose = Pose2D()
    pose.x, pose.y, pose.theta = *p
    return pose

def convert_ref_scan(ref_scan):
    return ScanWithPose(ref_scan[1], pose_from_tuple(ref_scan[0]))

def convert_ref_scans(scans):
    return map(convert_ref_scan, scans)
