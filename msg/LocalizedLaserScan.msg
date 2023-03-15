# Single scan from a planar laser range-finder and the pose of the robot

sensor_msgs/LaserScan scan
# Scanner pose in robot base_frame
geometry_msgs/TransformStamped scanner_offset
# Robot base_frame pose in a global (map) frame
geometry_msgs/PoseWithCovarianceStamped pose
