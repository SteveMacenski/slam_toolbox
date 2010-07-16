/*********************************************************//**
 * \file
 *
 * A simple test harness for KartoScanMatcher
 *
 * \author Bhaskara Marthi
 ************************************************************/

#include <karto_scan_matcher/karto_scan_matcher.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <boost/circular_buffer.hpp>
#include <string>
#include <ros/time.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace karto_scan_matcher
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
using std::string;
using std::vector;

using gm::Pose2D;

typedef boost::circular_buffer<ScanWithPose> ScanBuffer;
typedef boost::mutex::scoped_lock Lock;

/************************************************************
 * Node class
 ************************************************************/

class ScanMatcherTest
{

public:

  ScanMatcherTest ();

private:

  /****************************************
   * Ops
   ****************************************/
  
  void scanCallback (const sm::LaserScan& scan);
  void attemptScanMatch (const ros::WallTimerEvent& e);
  Pose2D perturbPose (const Pose2D& pose);
  
  /****************************************
   * Params
   ****************************************/

  const unsigned scan_history_length_;
  const double noise_level_;


  /****************************************
   * State
   ****************************************/

  ScanBuffer scans_;

  /****************************************
   * Associated objects
   ****************************************/
 
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::WallTimer scan_match_timer_;
  boost::mutex mutex_;
  boost::shared_ptr<KartoScanMatcher> matcher_;

};

/************************************************************
 * Init
 ************************************************************/

template <class T>
T getPrivateParam(const string& name, const T& default_value)
{
  ros::NodeHandle nh("~");
  T value;
  nh.param(name, value, default_value);
  return value;
}


ScanMatcherTest::ScanMatcherTest () :
  scan_history_length_(getPrivateParam<int>("scan_history_length", 10)), 
  noise_level_(getPrivateParam<double>("noise_level", 0.1)),
  scans_(scan_history_length_+1),
  scan_sub_(nh_.subscribe("base_scan", 10, &ScanMatcherTest::scanCallback, this)),
  scan_match_timer_(nh_.createWallTimer(ros::WallDuration(5.0), &ScanMatcherTest::attemptScanMatch, this))
{
  ROS_ASSERT (scan_history_length_ > 1);
}

/************************************************************
 * Callbacks
 ************************************************************/

void ScanMatcherTest::scanCallback (const sm::LaserScan& scan)
{
  // If this is the first scan, initialize the matcher
  if (!matcher_.get()) {
    matcher_.reset(new KartoScanMatcher(scan, tf_, 0.3, 0.01));
    ROS_INFO ("Initialized matcher");
  }


  try {

    // Figure out current base position
    gm::PoseStamped identity;
    identity.header.frame_id = "base_link";
    identity.header.stamp = ros::Time();
    identity.pose.orientation.w = 1.0;
    identity.pose.orientation.x = identity.pose.orientation.y = identity.pose.orientation.z = 0.0;
    gm::PoseStamped map_pose;
    tf_.transformPose("/map", identity, map_pose);
  
    // Save it and the scan
    Lock lock(mutex_);
    ScanWithPose s;
    s.scan = scan;
    s.pose.x = map_pose.pose.position.x;
    s.pose.y = map_pose.pose.position.y;
    s.pose.theta = tf::getYaw(map_pose.pose.orientation);
    scans_.push_back(s);
  }
  catch (tf::LookupException& e) {
    ROS_INFO ("Not saving scan due to a tf lookup exception");
  }
}



/************************************************************
 * Scan matching
 ************************************************************/



Pose2D ScanMatcherTest::perturbPose (const Pose2D& pose)
{
  Pose2D perturbed;
  perturbed.x = pose.x + noise_level_;
  perturbed.y = pose.y - noise_level_;
  perturbed.theta = pose.theta + noise_level_;
  return perturbed;
}

void ScanMatcherTest::attemptScanMatch (const ros::WallTimerEvent& e)
{
  ROS_INFO ("In attemptScanMatch.  Scan buffer has %zu elements", scans_.size());
  if (scans_.size() > scan_history_length_) {

    Lock lock(mutex_);
    Pose2D true_pose = scans_.back().pose;
    Pose2D perturbed_pose = perturbPose(true_pose);

    vector<ScanWithPose> reference_scans(scans_.begin(), scans_.end()-1);

    ROS_INFO_STREAM ("True pose is " << true_pose << " and perturbed pose is " << perturbed_pose);
    ROS_INFO_STREAM ("First reference scan is at " << reference_scans.begin()->pose << 
                     " and last one is at " << (reference_scans.end()-1)->pose);
    Pose2D estimated_pose = matcher_->scanMatch(scans_.back().scan, perturbed_pose, reference_scans).first;
    ROS_INFO_STREAM ("Estimated pose is " << estimated_pose);
  }
}


} // namespace karto_scan_matcher



int main (int argc, char** argv)
{
  ros::init(argc, argv, "test_scan_matcher");
  karto_scan_matcher::ScanMatcherTest n;
  ros::spin();
  
}


