/*********************************************************//**
 * \file
 * 
 * Implementation of karto_scan_matcher.h
 * 
 * \author Bhaskara Marthi
 ************************************************************/

#include <karto_scan_matcher/karto_scan_matcher.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

namespace karto_scan_matcher
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
using std::string;
using std::vector;
using boost::bind;
using boost::lexical_cast;


// Since Karto has a global namespace for sensors, we need to autogenerate unique ones of these
static boost::mutex static_name_mutex;
static unsigned name_suffix;


/************************************************************
 * Conversion
 ************************************************************/

karto::LocalizedRangeScan* convertLaserScan (karto::LaserRangeFinder* laser, const sm::LaserScan& scan, 
                                             const gm::Pose2D& pose)
{
  // Convert to Karto
  karto::Pose2 karto_pose(pose.x, pose.y, pose.theta);
  vector<kt_double> readings(scan.ranges.begin(), scan.ranges.end());

  karto::LocalizedRangeScan* localized = new karto::LocalizedRangeScan(laser->GetName(), readings);
  localized->SetOdometricPose(karto_pose);
  localized->SetCorrectedPose(karto_pose);
  return localized;
}

karto::LocalizedRangeScan* convertLaserScan(karto::LaserRangeFinder* laser, const ScanWithPose& scan)
{
  return convertLaserScan(laser, scan.scan, scan.pose);
}


/************************************************************
 * Initialization 
 ************************************************************/

KartoScanMatcher::KartoScanMatcher (const sm::LaserScan& init_scan, const tf::TransformListener& tf,
                                    const double search_space_size, const double search_grid_resolution)
{
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<btTransform> laser_to_base;
  ident.setIdentity();
  ident.frame_id_ = init_scan.header.frame_id;
  ident.stamp_ = ros::Time();
  
  if (!tf.waitForTransform("base_link", ident.frame_id_, ros::Time::now(), ros::Duration(10.0))) {
    ROS_FATAL_STREAM ("Didn't get a transform between base_link and " << ident.frame_id_ <<
                      " in time during KartoScanMatcher initialization");
    ROS_BREAK();
  }
  
  tf.transformPose("base_link", ident, laser_to_base);

  gm::Pose2D laser_pose;
  laser_pose.x = laser_to_base.getOrigin().x();
  laser_pose.y = laser_to_base.getOrigin().y();
  laser_pose.theta = tf::getYaw(laser_to_base.getRotation());

  vector<double> search_sizes;
  search_sizes.push_back(search_space_size);
  vector<double> search_resolutions;
  search_resolutions.push_back(search_grid_resolution);  
  initialize(init_scan, laser_pose, search_sizes, search_resolutions);
}

KartoScanMatcher::KartoScanMatcher (const sm::LaserScan& init_scan, const gm::Pose2D& laser_pose,
                                    const double search_space_size, const double search_grid_resolution)
{
  DoubleVector search_sizes;
  search_sizes.push_back(search_space_size);
  DoubleVector search_resolutions;
  search_resolutions.push_back(search_grid_resolution);  
  initialize(init_scan, laser_pose, search_sizes, search_resolutions);
}

KartoScanMatcher::KartoScanMatcher (const sensor_msgs::LaserScan& init_scan, const geometry_msgs::Pose2D& laser_pose,
                                    const DoubleVector& search_sizes, const DoubleVector& search_resolutions)
{
  initialize(init_scan, laser_pose, search_sizes, search_resolutions);
}

void KartoScanMatcher::initialize (const sm::LaserScan& init_scan, const gm::Pose2D& laser_pose,
                                   const DoubleVector& search_sizes, const DoubleVector& search_resolutions)
{
  string suffix;
  {
    boost::mutex::scoped_lock l(static_name_mutex);
    suffix =  boost::lexical_cast<string>(name_suffix++);
  }
  
  ROS_DEBUG_NAMED ("karto", "Initializing scan matcher(s) using scan %u", init_scan.header.seq);
  dataset_.reset(new karto::Dataset());
  const size_t num_matchers = search_sizes.size();
  if (num_matchers == 0)
    ROS_WARN_NAMED ("karto", "Zero sets of matcher parameters given to KartoScanMatcher: almost surely an error");
  ROS_ASSERT (search_resolutions.size() == num_matchers);
  matchers_.resize(num_matchers);
  mappers_.resize(num_matchers);
  for (size_t i=0; i<num_matchers; i++) {
    if (i>0) {
      if (search_sizes[i] >= search_sizes[i-1])
        ROS_WARN_NAMED ("karto", "Matcher %zu search size was %f and %zu was %f, which look out of order",
                        i-1, search_sizes[i-1], i, search_sizes[i]);
      if (search_resolutions[i] >= search_resolutions[i-1])
        ROS_WARN_NAMED ("karto", "Matcher %zu search resolution was %f and %zu was %f, which look out of order",
                        i-1, search_resolutions[i-1], i, search_sizes[i]);
    }
    mappers_[i].reset(new karto::Mapper());
    matchers_[i].reset(karto::ScanMatcher::Create(mappers_[i].get(), search_sizes[i], search_resolutions[i],
                                                  DEFAULT_SMEAR_DEVIATION, DEFAULT_RANGE_THRESHOLD));
    ROS_DEBUG_NAMED ("karto", "Search space size for matcher %zu is %.2f and resolution is %.2f",
                     i, search_sizes[i], search_resolutions[i]);
  }


  // This is copied over from slam_karto.cpp
  
  // Create a laser range finder device and copy in data from the first
  // scan
  string name = init_scan.header.frame_id;
  laser_ = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name+suffix));
  laser_->SetOffsetPose(karto::Pose2(laser_pose.x, laser_pose.y, laser_pose.theta));
  laser_->SetMinimumRange(init_scan.range_min);
  laser_->SetMaximumRange(init_scan.range_max);
  laser_->SetMinimumAngle(init_scan.angle_min);
  laser_->SetMaximumAngle(init_scan.angle_max);
  laser_->SetAngularResolution(init_scan.angle_increment);
  // TODO: expose this, and many other parameters
  //laser_->SetRangeThreshold(12.0);
  dataset_->Add(laser_);
}


/************************************************************
 * Scan matching
 ************************************************************/

gm::Pose2D subtractLaserOffset (const karto::Pose2& pose, const karto::Pose2& offset)
{
  btTransform laser_to_base(tf::createQuaternionFromYaw(offset.GetHeading()),
                            btVector3(offset.GetX(), offset.GetY(), 0.0));
  btTransform laser_to_map(tf::createQuaternionFromYaw(pose.GetHeading()),
                           btVector3(pose.GetX(), pose.GetY(), 0.0));
  btTransform base_to_map = laser_to_map*laser_to_base.inverse();
  gm::Pose2D result;
  result.x = base_to_map.getOrigin().x();
  result.y = base_to_map.getOrigin().y();
  result.theta = tf::getYaw(base_to_map.getRotation());
  return result;
}

typedef boost::shared_ptr<karto::LocalizedRangeScan> ScanPtr;
ScanPtr wrapBareScanPtr (karto::LocalizedRangeScan* scan)
{
  return ScanPtr(scan);
}


ScanMatchResult KartoScanMatcher::scanMatch (const sm::LaserScan& scan, const gm::Pose2D& pose, 
                                        const vector<ScanWithPose>& reference_scans) const
{
  ROS_DEBUG_NAMED ("karto", "Converting to Karto and matching");
  
  karto::LocalizedRangeScanVector localized_reference_scans;
  transform(reference_scans.begin(), reference_scans.end(), back_inserter(localized_reference_scans),
            bind(convertLaserScan, laser_, _1));

  // RAII objects to ensure the above pointers get freed at end of this scope
  vector<ScanPtr> scan_ptrs(localized_reference_scans.size());
  transform(localized_reference_scans.begin(), localized_reference_scans.end(),
            scan_ptrs.begin(), wrapBareScanPtr);

  gm::Pose2D current_estimate = pose;
  double last_response = -42.42;

  // Repeatedly match with each matcher, using the previous matcher's estimate to initialize
  BOOST_FOREACH (const MatcherPtr matcher, matchers_) {

    karto::LocalizedRangeScan* localized_scan = convertLaserScan(laser_, scan, current_estimate);

    // To make sure it gets freed we wrap it in a shared pointer
    // In later versions of Karto, the 'dataset' object is being used to memory-manage such pointers
    // I'm keeping it this way for now because the dataset only frees things when it goes out of scope
    // I could make it local to this function, but then it'd have to reinitialize the laser scan object
    // every time.  There seems to be no other effect of adding a laser scan to a dataset, so this should
    // be fine.
    ScanPtr scan_ptr(localized_scan); 

    ROS_DEBUG_NAMED ("karto", "  Current estimate is %.2f, %.2f, %.2f, Calling scan matcher",
                     current_estimate.x, current_estimate.y, current_estimate.theta);
    karto::Pose2 mean;
    karto::Matrix3 cov;
    last_response = matcher->MatchScan(localized_scan, localized_reference_scans, mean, cov);
    current_estimate = subtractLaserOffset(mean, laser_->GetOffsetPose());
    ROS_DEBUG_NAMED ("karto", "  Response was %.4f.", last_response);
  }
  
  ROS_DEBUG_NAMED ("karto", "Returning result %.2f, %.2f, %.2f",
                   current_estimate.x, current_estimate.y, current_estimate.theta);
  return ScanMatchResult(current_estimate, last_response);
}




} // namespace karto_scan_matcher
