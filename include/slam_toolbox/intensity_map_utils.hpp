#ifndef SLAM_TOOLBOX_INTENSITY_MAP_UTILS_HPP
#define SLAM_TOOLBOX_INTENSITY_MAP_UTILS_HPP

#include <cmath>
#include "karto_sdk/Karto.h"  
#include "slam_toolbox/intensity_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#ifndef MAP_IDX
#define MAP_IDX(width, x, y) ((y) * (width) + (x))
#endif

namespace slam_toolbox {

/**
 * @brief Updates an intensity map from a LocalizedRangeScan and an OccupancyGrid.
 *
 * For each ray in the scan, its endpoint is calculated in world coordinates from the sensor's
 * pose and angle (calculated using the minimum angle and angular resolution of the sensor).
 * That endpoint is then converted to grid coordinates using the occupancy grid's CoordinateConverter.
 * If the cell in the occupancy grid has the value GridStates_Occupied (100), the intensity value (converted to an 8-bit integer, rounded) is copied to that same cell
 * in the intensity grid.
 *
 * @param scan           Pointer to LocalizedRangeScan (ranges and intensities)
 * @param occ_grid       Pointer to occupancy grid (built) to be used for sizes and conversion
 * @param intensity_grid Reference to IntensityGrid object to update
 */
inline void updateIntensityGridFromScan(const karto::LocalizedRangeScan* scan,
                                          const karto::OccupancyGrid* occ_grid,
                                          slam_toolbox::IntensityGrid & intensity_grid)
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
        if(newValue < 45){ //minIntensity
            continue;
        }
        // TODO: scale the values if they are greater than 255.
        // Average the values ​​and limit to 255 to avoid overflow
        kt_int16u avg = static_cast<kt_int16u>(currentValue) + static_cast<kt_int16u>(newValue);
        avg /=2;
        intensity_grid.GetDataPointer()[idx] = static_cast<kt_int8u>(avg);
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
