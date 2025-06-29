#ifndef SLAM_TOOLBOX_INTENSITY_MAP_UTILS_HPP
#define SLAM_TOOLBOX_INTENSITY_MAP_UTILS_HPP

#include <cmath>
#include "karto_sdk/Karto.h"  
#include "slam_toolbox/intensity_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <algorithm>

#ifndef MAP_IDX
#define MAP_IDX(width, x, y) ((y) * (width) + (x))
#endif

namespace slam_toolbox {

enum class IntensityFusionStrategy {
    MEAN,
    REPLACE,
    WEIGHTED_MEAN,
    MAX
};

/**
 * @brief Converts a user-supplied string to the corresponding IntensityFusionStrategy enum.
 *
 * This function allows the fusion strategy to be specified as a string (e.g. "mean", "replace", "max", "weighted_mean"),
 * ignoring case. If the string does not match any known strategy, it defaults to MEAN and logs a warning.
 *
 * @param strategy_str The user-provided fusion strategy string (case-insensitive).
 * @param node         Shared pointer to the rclcpp node for logging warnings.
 * @return IntensityFusionStrategy corresponding to the string, or MEAN if unrecognized.
 */
inline IntensityFusionStrategy parseFusionStrategy(
    const std::string & strategy_str)
{
    std::string s = strategy_str;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower); // Convert to lower case

    if (s == "mean")          return IntensityFusionStrategy::MEAN;
    if (s == "replace")       return IntensityFusionStrategy::REPLACE;
    if (s == "weighted_mean") return IntensityFusionStrategy::WEIGHTED_MEAN;
    if (s == "max")           return IntensityFusionStrategy::MAX;

    return IntensityFusionStrategy::MEAN;
}

/**
 * @brief Updates an intensity map from a LocalizedRangeScan and an OccupancyGrid,
 *        combining new intensity measurements with existing values according to the specified strategy.
 *
 * For each ray in the scan, its endpoint is calculated in world coordinates from the sensor's
 * pose and angle. That endpoint is then converted to grid coordinates using the occupancy grid's CoordinateConverter.
 * If the cell in the occupancy grid is marked as occupied, the intensity value is fused with the current
 * value in the intensity grid according to the selected fusion strategy.
 *
 * @param scan                  Pointer to LocalizedRangeScan (ranges and intensities)
 * @param occ_grid              Pointer to occupancy grid to be used for sizes and conversion
 * @param intensity_grid        Reference to IntensityGrid object to update
 * @param min_intensity_threshold  Minimum intensity value to consider (to avoid noise)
 * @param fusion_strategy       Strategy to fuse new intensity readings with existing grid values:
 *                             - MEAN: average the current and new values (default)
 *                             - REPLACE: use the new value directly
 *                             - WEIGHTED_MEAN: weighted average (see weighted_mean_alpha)
 *                             - MAX: take the maximum of both values
 * @param weighted_mean_alpha   Weight of the existing value in WEIGHTED_MEAN strategy (default: 0.8).
 *                             Ignored for other strategies.
 */
inline void updateIntensityGridFromScan(const karto::LocalizedRangeScan* scan,
                                          const karto::OccupancyGrid* occ_grid,
                                          slam_toolbox::IntensityGrid & intensity_grid,
                                          double min_intensity_threshold,
                                          const std::string & fusion_strategy_str = "mean",
                                          double weighted_mean_alpha = 0.8)
{

  const kt_double* ranges = scan->GetRangeReadings();
  const kt_double* intensities = scan->GetIntensityReadings();
  kt_int32u numReadings = scan->GetNumberOfRangeReadings();

  // Get sensor params
  karto::LaserRangeFinder* laser = scan->GetLaserRangeFinder();
  if (!laser)
    return;
  kt_double minAngle = laser->GetMinimumAngle();
  kt_double angularRes = laser->GetAngularResolution();

  // Get sensor pose (in world coordinates)
  karto::Pose2 scanPose = scan->GetSensorPose();

  // Process each ray
  for (kt_int32u i = 0; i < numReadings; i++) {
    kt_double r = ranges[i];
    // If range is not finite, skip it
    if (!std::isfinite(r))
      continue;

    // Global ray angle calculation
    kt_double angle = scanPose.GetHeading() + minAngle + i * angularRes;

    // Endpoint in world coordinates
    karto::Vector2<kt_double> endpoint;
    endpoint.SetX(scanPose.GetX() + r * cos(angle));
    endpoint.SetY(scanPose.GetY() + r * sin(angle));

    // Convert the endpoint to grid coordinates using the occupancy grid's CoordinateConverter
    karto::Vector2<kt_int32s> gridIndex = occ_grid->GetCoordinateConverter()->WorldToGrid(endpoint);
    if (!occ_grid->IsValidGridIndex(gridIndex))
      continue;
    kt_int32s idx = occ_grid->GridIndex(gridIndex);

    // If the cell in the occupancy grid is marked as occupied (value 100)
    if (occ_grid->GetDataPointer()[idx] == karto::GridStates_Occupied) {
        // Get the current value of the intensity grid cell
        kt_int8u currentValue = intensity_grid.GetDataPointer()[idx];
        // Convert the beam intensity to an 8-bit integer value (rounded)
        kt_int8u newValue = static_cast<kt_int8u>(std::round(intensities[i]));
        if(newValue < min_intensity_threshold){
            continue;
        }
        // Values not higher than 255
        if(newValue > 255)
          newValue = 255;
        
        // Fusion strategy
        kt_int16u fusedValue = 0;
        IntensityFusionStrategy fusion_strategy = parseFusionStrategy(fusion_strategy_str);
        switch (fusion_strategy) {
            case IntensityFusionStrategy::MEAN:
                fusedValue = static_cast<kt_int8u>((static_cast<kt_int16u>(currentValue) + newValue) / 2);
                break;
            case IntensityFusionStrategy::REPLACE:
                fusedValue = newValue;
                break;
            case IntensityFusionStrategy::WEIGHTED_MEAN:
                fusedValue = static_cast<kt_int8u>(weighted_mean_alpha * currentValue + (1.0 - weighted_mean_alpha) * newValue);
                break;
            case IntensityFusionStrategy::MAX:
                fusedValue = std::max(currentValue, newValue);
                break;
        }
        intensity_grid.GetDataPointer()[idx] = fusedValue;        
      }
  }
}

} // namespace slam_toolbox

namespace vis_utils {

    /**
     * @brief Convert IntensityGrid to a OccupancyGrid (nav_msgs::msg::OccupancyGrid).
     *
     * It is assumed that the IntensityGrid has the same dimensions, offset and resolution
     * than the occupancy grid, but in each cell stores the intensity value.
     *
     * @param int_grid Pointer to intensity map (IntensityGrid)
     * @param map      Reference to occuppancy grid to be filled/updated
     */
    inline void toNavIntensityMap(const slam_toolbox::IntensityGrid * int_grid,
                                  nav_msgs::msg::OccupancyGrid & map)
    {
      // Get dimensions and offset of the intensity grid.
      kt_int32s width = int_grid->GetWidth();
      kt_int32s height = int_grid->GetHeight();
      karto::Vector2<kt_double> offset = int_grid->GetCoordinateConverter()->GetOffset();
    
      // If the dimensions or offset do not match the current map, it is reconfigured.
      if (map.info.width != static_cast<unsigned int>(width) ||
          map.info.height != static_cast<unsigned int>(height) ||
          map.info.origin.position.x != offset.GetX() ||
          map.info.origin.position.y != offset.GetY())
      {
        map.info.origin.position.x = offset.GetX();
        map.info.origin.position.y = offset.GetY();
        // The orientation is assumed to be zero.
        map.info.origin.orientation.x = 0.0;
        map.info.origin.orientation.y = 0.0;
        map.info.origin.orientation.z = 0.0;
        map.info.origin.orientation.w = 1.0;
        map.info.width = width;
        map.info.height = height;
        // The data vector is resized.
        map.data.resize(width * height);
      }
    
      // Loop through each cell and copy the intensity value
      for (kt_int32s y = 0; y < height; y++) {
        for (kt_int32s x = 0; x < width; x++) {
          // The intensity value in the cell is obtained
          kt_int8u intensity = int_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
          // Assigned to the map
          map.data[MAP_IDX(width, x, y)] = static_cast<int8_t>(intensity);
        }
      }
    }
    
}  // namespace vis_utils

#endif // SLAM_TOOLBOX_INTENSITY_MAP_UTILS_HPP
