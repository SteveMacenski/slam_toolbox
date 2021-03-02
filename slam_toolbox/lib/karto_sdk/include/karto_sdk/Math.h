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

#ifndef karto_sdk_MATH_H
#define karto_sdk_MATH_H

#include <assert.h>
#include <math.h>
#include <limits>

#include <karto_sdk/Types.h>

namespace karto
{
  /**
   * Platform independent pi definitions
   */
  const kt_double KT_PI         =  3.14159265358979323846;  // The value of PI
  const kt_double KT_2PI        =  6.28318530717958647692;  // 2 * PI
  const kt_double KT_PI_2       =  1.57079632679489661923;  // PI / 2
  const kt_double KT_PI_180     =  0.01745329251994329577;  // PI / 180
  const kt_double KT_180_PI     = 57.29577951308232087685;  // 180 / PI

  /**
   * Lets define a small number!
   */
  const kt_double KT_TOLERANCE  = 1e-06;

  /**
     * Lets define max value of kt_int32s (int32_t) to use it to mark invalid scans
     */

  const kt_int32s INVALID_SCAN = std::numeric_limits<kt_int32s>::max();

  namespace math
  {
    /**
     * Converts degrees into radians
     * @param degrees
     * @return radian equivalent of degrees
     */
    inline kt_double DegreesToRadians(kt_double degrees)
    {
      return degrees * KT_PI_180;
    }

    /**
     * Converts radians into degrees
     * @param radians
     * @return degree equivalent of radians
     */
    inline kt_double RadiansToDegrees(kt_double radians)
    {
      return radians * KT_180_PI;
    }

    /**
     * Square function
     * @param value
     * @return square of value
     */
    template<typename T>
    inline T Square(T value)
    {
      return (value * value);
    }

    /**
     * Round function
     * @param value
     * @return rounds value to the nearest whole number (as double)
     */
    inline kt_double Round(kt_double value)
    {
      return value >= 0.0 ? floor(value + 0.5) : ceil(value - 0.5);
    }

    /**
     * Binary minimum function
     * @param value1
     * @param value2
     * @return the lesser of value1 and value2
     */
    template<typename T>
    inline const T& Minimum(const T& value1, const T& value2)
    {
      return value1 < value2 ? value1 : value2;
    }

    /**
     * Binary maximum function
     * @param value1
     * @param value2
     * @return the greater of value1 and value2
     */
    template<typename T>
    inline const T& Maximum(const T& value1, const T& value2)
    {
      return value1 > value2 ? value1 : value2;
    }

    /**
     * Clips a number to the specified minimum and maximum values.
     * @param n number to be clipped
     * @param minValue minimum value
     * @param maxValue maximum value
     * @return the clipped value
     */
    template<typename T>
    inline const T& Clip(const T& n, const T& minValue, const T& maxValue)
    {
      return Minimum(Maximum(n, minValue), maxValue);
    }

    /**
     * Checks whether two numbers are equal within a certain tolerance.
     * @param a
     * @param b
     * @return true if a and b differ by at most a certain tolerance.
     */
    inline kt_bool DoubleEqual(kt_double a, kt_double b)
    {
      double delta = a - b;
      return delta < 0.0 ? delta >= -KT_TOLERANCE : delta <= KT_TOLERANCE;
    }

    /**
     * Checks whether value is in the range [0;maximum)
     * @param value
     * @param maximum
     */
    template<typename T>
    inline kt_bool IsUpTo(const T& value, const T& maximum)
    {
      return (value >= 0 && value < maximum);
    }

    /**
     * Checks whether value is in the range [0;maximum)
     * Specialized version for unsigned int (kt_int32u)
     * @param value
     * @param maximum
     */
    template<>
    inline kt_bool IsUpTo<kt_int32u>(const kt_int32u& value, const kt_int32u& maximum)
    {
      return (value < maximum);
    }


    /**
     * Checks whether value is in the range [a;b]
     * @param value
     * @param a
     * @param b
     */
    template<typename T>
    inline kt_bool InRange(const T& value, const T& a, const T& b)
    {
      return (value >= a && value <= b);
    }

    /**
     * Normalizes angle to be in the range of [-pi, pi]
     * @param angle to be normalized
     * @return normalized angle
     */
    inline kt_double NormalizeAngle(kt_double angle)
    {
      while (angle < -KT_PI)
      {
        if (angle < -KT_2PI)
        {
          angle += (kt_int32u)(angle / -KT_2PI) * KT_2PI;
        }
        else
        {
          angle += KT_2PI;
        }
      }

      while (angle > KT_PI)
      {
        if (angle > KT_2PI)
        {
          angle -= (kt_int32u)(angle / KT_2PI) * KT_2PI;
        }
        else
        {
          angle -= KT_2PI;
        }
      }

      assert(math::InRange(angle, -KT_PI, KT_PI));

      return angle;
    }

    /**
     * Returns an equivalent angle to the first parameter such that the difference
     * when the second parameter is subtracted from this new value is an angle
     * in the normalized range of [-pi, pi], i.e. abs(minuend - subtrahend) <= pi.
     * @param minuend
     * @param subtrahend
     * @return normalized angle
     */
    inline kt_double NormalizeAngleDifference(kt_double minuend, kt_double subtrahend)
    {
      while (minuend - subtrahend < -KT_PI)
      {
        minuend += KT_2PI;
      }

      while (minuend - subtrahend > KT_PI)
      {
        minuend -= KT_2PI;
      }

      return minuend;
    }

    /**
     * Align a value to the alignValue.
     * The alignValue should be the power of two (2, 4, 8, 16, 32 and so on)
     * @param value
     * @param alignValue
     * @return aligned value
     */
    template<class T>
    inline T AlignValue(size_t value, size_t alignValue = 8)
    {
      return static_cast<T> ((value + (alignValue - 1)) & ~(alignValue - 1));
    }
  }  // namespace math

}  // namespace karto

#endif  // karto_sdk_MATH_H
