/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "karto_sdk/Karto.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(karto::NonCopyable);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Object);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Sensor);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::SensorData);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Name);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::LaserRangeScan);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::LocalizedRangeScan);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::CustomData);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Module);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::LaserRangeFinder);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Dataset);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::LookupArray);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::SensorManager);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::AbstractParameter);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::ParameterEnum);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Parameters);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::ParameterManager);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Parameter<kt_double >);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Parameter<karto::Pose2>);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Parameter<kt_bool>);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Parameter<kt_int32u>);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Parameter<kt_int32s>);
BOOST_CLASS_EXPORT_IMPLEMENT(karto::Parameter<std::string>);
namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  SensorManager* SensorManager::GetInstance()
  {
    static Singleton<SensorManager> sInstance;
    return sInstance.Get();
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Object::Object()
    : m_pParameterManager(new ParameterManager())
  {
  }

  Object::Object(const Name& rName)
    : m_Name(rName)
    , m_pParameterManager(new ParameterManager())
  {
  }

  Object::~Object()
  {
    delete m_pParameterManager;
    m_pParameterManager = NULL;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Module::Module(const std::string& rName)
    : Object(rName)
  {
  }

  Module::~Module()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Sensor::Sensor(const Name& rName)
    : Object(rName)
  {
    m_pOffsetPose = new Parameter<Pose2>("OffsetPose", Pose2(), GetParameterManager());
  }

  Sensor::~Sensor()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  SensorData::SensorData(const Name& rSensorName)
    : Object()
    , m_StateId(-1)
    , m_UniqueId(-1)
    , m_SensorName(rSensorName)
    , m_Time(0.0)
  {
  }

  SensorData::~SensorData()
  {
    forEach(CustomDataVector, &m_CustomData)
    {
      delete *iter;
    }

    m_CustomData.clear();
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void CellUpdater::operator() (kt_int32u index)
  {
    kt_int8u* pDataPtr = m_pOccupancyGrid->GetDataPointer();
    kt_int32u* pCellPassCntPtr = m_pOccupancyGrid->m_pCellPassCnt->GetDataPointer();
    kt_int32u* pCellHitCntPtr = m_pOccupancyGrid->m_pCellHitsCnt->GetDataPointer();

    m_pOccupancyGrid->UpdateCell(&pDataPtr[index], pCellPassCntPtr[index], pCellHitCntPtr[index]);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  std::ostream& operator << (std::ostream& rStream, Exception& rException)
  {
    rStream << "Error detect: " << std::endl;
    rStream << " ==> error code: " << rException.GetErrorCode() << std::endl;
    rStream << " ==> error message: " << rException.GetErrorMessage() << std::endl;

    return rStream;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  const PointVectorDouble LaserRangeFinder::GetPointReadings(LocalizedRangeScan* pLocalizedRangeScan,
                                                             CoordinateConverter* pCoordinateConverter,
                                                             kt_bool ignoreThresholdPoints,
                                                             kt_bool flipY) const
  {
    PointVectorDouble pointReadings;

    Pose2 scanPose = pLocalizedRangeScan->GetSensorPose();

    // compute point readings
    kt_int32u beamNum = 0;
    kt_double* pRangeReadings = pLocalizedRangeScan->GetRangeReadings();
    for (kt_int32u i = 0; i < m_NumberOfRangeReadings; i++, beamNum++)
    {
      kt_double rangeReading = pRangeReadings[i];

      if (ignoreThresholdPoints)
      {
        if (!math::InRange(rangeReading, GetMinimumRange(), GetRangeThreshold()))
        {
          continue;
        }
      }
      else
      {
        rangeReading = math::Clip(rangeReading, GetMinimumRange(), GetRangeThreshold());
      }

      kt_double angle = scanPose.GetHeading() + GetMinimumAngle() + beamNum * GetAngularResolution();

      Vector2<kt_double> point;

      point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
      point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

      if (pCoordinateConverter != NULL)
      {
        Vector2<kt_int32s> gridPoint = pCoordinateConverter->WorldToGrid(point, flipY);
        point.SetX(gridPoint.GetX());
        point.SetY(gridPoint.GetY());
      }

      pointReadings.push_back(point);
    }

    return pointReadings;
  }

  kt_bool LaserRangeFinder::Validate(SensorData* pSensorData)
  {
    LaserRangeScan* pLaserRangeScan = dynamic_cast<LaserRangeScan*>(pSensorData);

    // verify number of range readings in LaserRangeScan matches the number of expected range readings
    if (pLaserRangeScan->GetNumberOfRangeReadings() != GetNumberOfRangeReadings())
    {
      std::cout << "LaserRangeScan contains " << pLaserRangeScan->GetNumberOfRangeReadings()
                << " range readings, expected " << GetNumberOfRangeReadings() << std::endl;
      return false;
    }

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void ParameterManager::Clear()
  {
    forEach(karto::ParameterVector, &m_Parameters)
    {
      delete *iter;
    }

    m_Parameters.clear();

    m_ParameterLookup.clear();
  }

  void ParameterManager::Add(AbstractParameter* pParameter)
  {
    if (pParameter != NULL && pParameter->GetName() != "")
    {
      if (m_ParameterLookup.find(pParameter->GetName()) == m_ParameterLookup.end())
      {
        m_Parameters.push_back(pParameter);

        m_ParameterLookup[pParameter->GetName()] = pParameter;
      }
      else
      {
        m_ParameterLookup[pParameter->GetName()]->SetValueFromString(pParameter->GetValueAsString());

        assert(false);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /*
  std::string LaserRangeFinder::LaserRangeFinderTypeNames[6] =
  {
    "Custom",

    "Sick_LMS100",
    "Sick_LMS200",
    "Sick_LMS291",

    "Hokuyo_UTM_30LX",
    "Hokuyo_URG_04LX"
  };
   */
}  // namespace karto
