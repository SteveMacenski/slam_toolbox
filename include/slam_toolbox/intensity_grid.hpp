#ifndef SLAM_TOOLBOX_INTENSITY_GRID_HPP
#define SLAM_TOOLBOX_INTENSITY_GRID_HPP

#include "karto_sdk/Karto.h"   // To use Grid y CoordinateConverter
#include <vector>

namespace slam_toolbox {

class IntensityGrid : public karto::Grid<kt_int8u>
{
public:
  /**
   * Creates an IntensityGrid with same size and resolution of the occupancy grid.
   * @param width number of columns
   * @param height number of rows
   * @param rOffset offset (origin) of the world
   * @param resolution map resolution
   */
  IntensityGrid(kt_int32s width, kt_int32s height, const karto::Vector2<kt_double> & rOffset, kt_double resolution)
    : karto::Grid<kt_int8u>(width, height)
  {
    // Configurate the CoordinateConverter of the grid with the same resolution and offset.
    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(rOffset);
    
    // Inicialize all cells to a 0
    Clear();
  }

  virtual ~IntensityGrid() {}

  /**
   * Set an intensity value to a cell
   * @param gridIndex index
   * @param intensity intensity value (p.e. 0 a 255)
   */
  inline void SetIntensity(kt_int32s gridIndex, kt_int8u intensity)
  {
    if (gridIndex >= 0 && gridIndex < GetDataSize()) {
      GetDataPointer()[gridIndex] = intensity;
    }
  }

  /**
   * Get intensity value of a cell
   * @param gridIndex cell index
   * @return intensity value
   */
  inline kt_int8u GetIntensity(kt_int32s gridIndex) const
  {
    if (gridIndex >= 0 && gridIndex < GetDataSize()) {
      return GetDataPointer()[gridIndex];
    }
    return 0;
  }
};

} // namespace slam_toolbox

#endif // SLAM_TOOLBOX_INTENSITY_GRID_HPP
