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

#ifndef KARTO_SDK__KARTO_H_
#define KARTO_SDK__KARTO_H_

#include <math.h>
#include <float.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <boost/thread.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/export.hpp>
#include <boost/type_traits/is_abstract.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/array.hpp>
#include <boost/version.hpp>

#include <string>
#include <fstream>
#include <limits>
#include <algorithm>
#include <map>
#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <shared_mutex>
#include <mutex>

#ifdef USE_POCO
#include <Poco/Mutex.h>
#endif

#include "Math.h"
#include "Macros.h"

#define KARTO_Object(name) \
  virtual const char * GetClassName() const {return #name;} \
  virtual kt_objecttype GetObjectType() const {return ObjectType_ ## name;}

typedef kt_int32u kt_objecttype;

const kt_objecttype ObjectType_None = 0x00000000;
const kt_objecttype ObjectType_Sensor = 0x00001000;
const kt_objecttype ObjectType_SensorData = 0x00002000;
const kt_objecttype ObjectType_CustomData = 0x00004000;
const kt_objecttype ObjectType_Misc = 0x10000000;

const kt_objecttype ObjectType_Drive = ObjectType_Sensor | 0x01;
const kt_objecttype ObjectType_LaserRangeFinder = ObjectType_Sensor | 0x02;
const kt_objecttype ObjectType_Camera = ObjectType_Sensor | 0x04;

const kt_objecttype ObjectType_DrivePose = ObjectType_SensorData | 0x01;
const kt_objecttype ObjectType_LaserRangeScan = ObjectType_SensorData | 0x02;
const kt_objecttype ObjectType_LocalizedRangeScan = ObjectType_SensorData | 0x04;
const kt_objecttype ObjectType_CameraImage = ObjectType_SensorData | 0x08;
const kt_objecttype ObjectType_LocalizedRangeScanWithPoints = ObjectType_SensorData | 0x16;

const kt_objecttype ObjectType_Header = ObjectType_Misc | 0x01;
const kt_objecttype ObjectType_Parameters = ObjectType_Misc | 0x02;
const kt_objecttype ObjectType_DatasetInfo = ObjectType_Misc | 0x04;
const kt_objecttype ObjectType_Module = ObjectType_Misc | 0x08;

namespace karto
{

/**
 * \defgroup OpenKarto OpenKarto Module
 */
/*@{*/

/**
 * Exception class. All exceptions thrown from Karto will inherit from this class or be of this class
 */
class KARTO_EXPORT Exception
{
public:
  /**
   * Constructor with exception message
   * @param rMessage exception message (default: "Karto Exception")
   * @param errorCode error code (default: 0)
   */
  Exception(const std::string & rMessage = "Karto Exception", kt_int32s errorCode = 0)  // NOLINT
  : m_Message(rMessage),
    m_ErrorCode(errorCode)
  {
  }

  /**
   * Copy constructor
   */
  Exception(const Exception & rException)
  : m_Message(rException.m_Message),
    m_ErrorCode(rException.m_ErrorCode)
  {
  }

  /**
   * Destructor
   */
  virtual ~Exception()
  {
  }

public:
  /**
   * Assignment operator
   */
  Exception & operator=(const Exception & rException)
  {
    m_Message = rException.m_Message;
    m_ErrorCode = rException.m_ErrorCode;

    return *this;
  }

public:
  /**
   * Gets the exception message
   * @return error message as string
   */
  const std::string & GetErrorMessage() const
  {
    return m_Message;
  }

  /**
   * Gets error code
   * @return error code
   */
  kt_int32s GetErrorCode()
  {
    return m_ErrorCode;
  }

public:
  /**
   * Write exception to output stream
   * @param rStream output stream
   * @param rException exception to write
   */
  friend KARTO_EXPORT std::ostream & operator<<(std::ostream & rStream, Exception & rException);

private:
  std::string m_Message;
  kt_int32s m_ErrorCode;
};    // class Exception

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Subclass this class to make a non-copyable class (copy
 * constructor and assignment operator are private)
 */
class KARTO_EXPORT NonCopyable
{
private:
  NonCopyable(const NonCopyable &) = delete;
  const NonCopyable & operator=(const NonCopyable &) = delete;

public:
  NonCopyable()
  {
  }

  virtual ~NonCopyable()
  {
  }

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
  }
};    // class NonCopyable

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Singleton class ensures only one instance of T is created
 */
template<class T>
class Singleton
{
public:
  /**
   * Constructor
   */
  Singleton()
  : m_pPointer(NULL)
  {
  }

  /**
   * Destructor
   */
  virtual ~Singleton()
  {
    delete m_pPointer;
  }

  /**
   * Gets the singleton
   * @return singleton
   */
  T * Get()
  {
#ifdef USE_POCO
    Poco::FastMutex::ScopedLock lock(m_Mutex);
#endif
    if (m_pPointer == NULL) {
      m_pPointer = new T;
    }

    return m_pPointer;
  }

private:
  T * m_pPointer;

#ifdef USE_POCO
  Poco::FastMutex m_Mutex;
#endif

private:
  Singleton(const Singleton &);
  const Singleton & operator=(const Singleton &);
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Functor
 */
class KARTO_EXPORT Functor
{
public:
  /**
   * Functor function
   */
  virtual void operator()(kt_int32u) {}
};    // Functor

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class AbstractParameter;

/**
 * Type declaration of AbstractParameter vector
 */
typedef std::vector<AbstractParameter *> ParameterVector;

/**
 * Parameter manager.
 */
class KARTO_EXPORT ParameterManager : public NonCopyable
{
public:
  /**
   * Default constructor
   */
  ParameterManager()
  {
  }

  /**
   * Destructor
   */
  virtual ~ParameterManager()
  {
    Clear();
  }

public:
  /**
   * Adds the parameter to this manager
   * @param pParameter
   */
  void Add(AbstractParameter * pParameter);

  /**
   * Gets the parameter of the given name
   * @param rName
   * @return parameter of given name
   */
  AbstractParameter * Get(const std::string & rName)
  {
    if (m_ParameterLookup.find(rName) != m_ParameterLookup.end()) {
      return m_ParameterLookup[rName];
    }

    std::cout << "Unknown parameter: " << rName << std::endl;

    return NULL;
  }

  /**
   * Clears the manager of all parameters
   */
  void Clear();

  /**
   * Gets all parameters
   * @return vector of all parameters
   */
  inline const ParameterVector & GetParameterVector() const
  {
    return m_Parameters;
  }

public:
  /**
   * Gets the parameter with the given name
   * @param rName
   * @return parameter of given name
   */
  AbstractParameter * operator()(const std::string & rName)
  {
    return Get(rName);
  }

  /**
   * Serialization: class ParameterManager
   */

private:
  ParameterManager(const ParameterManager &);
  const ParameterManager & operator=(const ParameterManager &);

private:
  ParameterVector m_Parameters;
  std::map<std::string, AbstractParameter *> m_ParameterLookup;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(NonCopyable);
    ar & BOOST_SERIALIZATION_NVP(m_Parameters);
    ar & BOOST_SERIALIZATION_NVP(m_ParameterLookup);
  }
};    // ParameterManager

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// valid names
// 'Test' -- no scope
// '/Test' -- no scope will be parsed to 'Test'
// '/scope/Test' - 'scope' scope and 'Test' name
// '/scope/name/Test' - 'scope/name' scope and 'Test' name
//
class Name
{
public:
  /**
   * Constructor
   */
  Name()
  {
  }

  /**
   * Constructor
   */
  Name(const std::string & rName)  // NOLINT
  {
    Parse(rName);
  }

  /**
   * Constructor
   */
  Name(const Name & rOther)
  : m_Scope(rOther.m_Scope), m_Name(rOther.m_Name)
  {
  }

  /**
   * Destructor
   */
  virtual ~Name()
  {
  }

public:
  /**
   * Gets the name of this name
   * @return name
   */
  inline const std::string & GetName() const
  {
    return m_Name;
  }

  /**
   * Sets the name
   * @param rName name
   */
  inline void SetName(const std::string & rName)
  {
    std::string::size_type pos = rName.find_last_of('/');
    if (pos != 0 && pos != std::string::npos) {
      throw Exception("Name can't contain a scope!");
    }

    m_Name = rName;
  }

  /**
   * Gets the scope of this name
   * @return scope
   */
  inline const std::string & GetScope() const
  {
    return m_Scope;
  }

  /**
   * Sets the scope of this name
   * @param rScope scope
   */
  inline void SetScope(const std::string & rScope)
  {
    m_Scope = rScope;
  }

  /**
   * Returns a string representation of this name
   * @return string representation of this name
   */
  inline std::string ToString() const
  {
    if (m_Scope.empty()) {
      return m_Name;
    } else {
      std::string name;
      name.append("/");
      name.append(m_Scope);
      name.append("/");
      name.append(m_Name);

      return name;
    }
  }

public:
  /**
   * Assignment operator.
   */
  Name & operator=(const Name & rOther)
  {
    if (&rOther != this) {
      m_Name = rOther.m_Name;
      m_Scope = rOther.m_Scope;
    }

    return *this;
  }

  /**
   * Equality operator.
   */
  kt_bool operator==(const Name & rOther) const
  {
    return (m_Name == rOther.m_Name) && (m_Scope == rOther.m_Scope);
  }

  /**
   * Inequality operator.
   */
  kt_bool operator!=(const Name & rOther) const
  {
    return !(*this == rOther);
  }

  /**
   * Less than operator.
   */
  kt_bool operator<(const Name & rOther) const
  {
    return this->ToString() < rOther.ToString();
  }

  /**
   * Write Name onto output stream
   * @param rStream output stream
   * @param rName to write
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Name & rName)
  {
    rStream << rName.ToString();
    return rStream;
  }

private:
  /**
   * Parse the given string into a Name object
   * @param rName name
   */
  void Parse(const std::string & rName)
  {
    std::string::size_type pos = rName.find_last_of('/');

    if (pos == std::string::npos) {
      m_Name = rName;
    } else {
      m_Scope = rName.substr(0, pos);
      m_Name = rName.substr(pos + 1, rName.size());

      // remove '/' from m_Scope if first!!
      if (m_Scope.size() > 0 && m_Scope[0] == '/') {
        m_Scope = m_Scope.substr(1, m_Scope.size());
      }
    }
  }

  /**
   * Validates the given string as a Name
   * @param rName name
   */
  void Validate(const std::string & rName)
  {
    if (rName.empty()) {
      return;
    }

    char c = rName[0];
    if (IsValidFirst(c)) {
      for (size_t i = 1; i < rName.size(); ++i) {
        c = rName[i];
        if (!IsValid(c)) {
          throw Exception(
                  "Invalid character in name. "
                  "Valid characters must be within the ranges "
                  "A-Z, a-z, 0-9, '/', '_' and '-'.");
        }
      }
    } else {
      throw Exception(
              "Invalid first character in name. "
              "Valid characters must be within the ranges A-Z, a-z, and '/'.");
    }
  }

  /**
   * Whether the character is valid as a first character (alphanumeric or /)
   * @param c character
   * @return true if the character is a valid first character
   */
  inline kt_bool IsValidFirst(char c)
  {
    return isalpha(c) || c == '/';
  }

  /**
   * Whether the character is a valid character (alphanumeric, /, _, or -)
   * @param c character
   * @return true if the character is valid
   */
  inline kt_bool IsValid(char c)
  {
    return isalnum(c) || c == '/' || c == '_' || c == '-';
  }

private:
  std::string m_Name;
  std::string m_Scope;
  /**
   * Serialization: class Name
   */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Name);
    ar & BOOST_SERIALIZATION_NVP(m_Scope);
  }
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Abstract base class for Karto objects.
 */
class KARTO_EXPORT Object : public NonCopyable
{
public:
  /**
   * Default constructor
   */
  Object();

  /**
   * Constructs an object with the given name
   * @param rName
   */
  Object(const Name & rName);  // NOLINT

  /**
   * Default constructor
   */
  virtual ~Object();

public:
  /**
   * Gets the name of this object
   * @return name
   */
  inline const Name & GetName() const
  {
    return m_Name;
  }

  /**
   * Gets the class name of this object
   * @return class name
   */
  virtual const char * GetClassName() const = 0;

  /**
   * Gets the type of this object
   * @return object type
   */
  virtual kt_objecttype GetObjectType() const = 0;

  /**
   * Gets the parameter manager of this dataset
   * @return parameter manager
   */
  virtual inline ParameterManager * GetParameterManager()
  {
    return m_pParameterManager;
  }

  /**
   * Gets the named parameter
   * @param rName name of parameter
   * @return parameter
   */
  inline AbstractParameter * GetParameter(const std::string & rName) const
  {
    return m_pParameterManager->Get(rName);
  }

  /**
   * Sets the parameter with the given name with the given value
   * @param rName name
   * @param value value
   */
  template<typename T>
  inline void SetParameter(const std::string & rName, T value);

  /**
   * Gets all parameters
   * @return parameters
   */
  inline const ParameterVector & GetParameters() const
  {
    return m_pParameterManager->GetParameterVector();
  }

  Object(const Object &);
  const Object & operator=(const Object &);

private:
  Name m_Name;
  ParameterManager * m_pParameterManager;
  /**
* Serialization: class Object
*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(NonCopyable);
    ar & BOOST_SERIALIZATION_NVP(m_pParameterManager);
    ar & BOOST_SERIALIZATION_NVP(m_Name);
  }
};
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Object)

/**
 * Type declaration of Object vector
 */
typedef std::vector<Object *> ObjectVector;
typedef std::map<kt_int32s, Object *> DataMap;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Whether the object is a sensor
 * @param pObject object
 * @return whether the object is a sensor
 */
inline kt_bool IsSensor(Object * pObject)
{
  return (pObject->GetObjectType() & ObjectType_Sensor) == ObjectType_Sensor;
}

/**
 * Whether the object is sensor data
 * @param pObject object
 * @return whether the object is sensor data
 */
inline kt_bool IsSensorData(Object * pObject)
{
  return (pObject->GetObjectType() & ObjectType_SensorData) == ObjectType_SensorData;
}

/**
 * Whether the object is a laser range finder
 * @param pObject object
 * @return whether the object is a laser range finder
 */
inline kt_bool IsLaserRangeFinder(Object * pObject)
{
  return (pObject->GetObjectType() & ObjectType_LaserRangeFinder) == ObjectType_LaserRangeFinder;
}

/**
 * Whether the object is a localized range scan
 * @param pObject object
 * @return whether the object is a localized range scan
 */
inline kt_bool IsLocalizedRangeScan(Object * pObject)
{
  return (pObject->GetObjectType() & ObjectType_LocalizedRangeScan) ==
         ObjectType_LocalizedRangeScan;
}

/**
 * Whether the object is a localized range scan with points
 * @param pObject object
 * @return whether the object is a localized range scan with points
 */
inline kt_bool IsLocalizedRangeScanWithPoints(Object * pObject)
{
  return (pObject->GetObjectType() & ObjectType_LocalizedRangeScanWithPoints) ==
         ObjectType_LocalizedRangeScanWithPoints;
}

/**
 * Whether the object is a Parameters object
 * @param pObject object
 * @return whether the object is a Parameters object
 */
inline kt_bool IsParameters(Object * pObject)
{
  return (pObject->GetObjectType() & ObjectType_Parameters) == ObjectType_Parameters;
}

/**
 * Whether the object is a DatasetInfo object
 * @param pObject object
 * @return whether the object is a DatasetInfo object
 */
inline kt_bool IsDatasetInfo(Object * pObject)
{
  return (pObject->GetObjectType() & ObjectType_DatasetInfo) == ObjectType_DatasetInfo;
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Abstract base class for Karto modules.
 */
class KARTO_EXPORT Module : public Object
{
public:
  // @cond EXCLUDE
  KARTO_Object(Module)
  // @endcond

public:
  /**
   * Construct a Module
   * @param rName module name
   */
  Module(const std::string & rName);  // NOLINT

  /**
   * Destructor
   */
  virtual ~Module();

public:
  /**
   * Reset the module
   */
  virtual void Reset() = 0;

  /**
   * Process an Object
   */
  virtual kt_bool Process(karto::Object *)
  {
    return false;
  }

private:
  Module(const Module &);
  const Module & operator=(const Module &);

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Object);
  }
};
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Module)

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Represents a size (width, height) in 2-dimensional real space.
 */
template<typename T>
class Size2
{
public:
  /**
   * Default constructor
   */
  Size2()
  : m_Width(0),
    m_Height(0)
  {
  }

  /**
   * Constructor initializing point location
   * @param width
   * @param height
   */
  Size2(T width, T height)
  : m_Width(width),
    m_Height(height)
  {
  }

  /**
   * Copy constructor
   * @param rOther
   */
  Size2(const Size2 & rOther)
  : m_Width(rOther.m_Width),
    m_Height(rOther.m_Height)
  {
  }

public:
  /**
   * Gets the width
   * @return the width
   */
  inline const T GetWidth() const
  {
    return m_Width;
  }

  /**
   * Sets the width
   * @param width
   */
  inline void SetWidth(T width)
  {
    m_Width = width;
  }

  /**
   * Gets the height
   * @return the height
   */
  inline const T GetHeight() const
  {
    return m_Height;
  }

  /**
   * Sets the height
   * @param height
   */
  inline void SetHeight(T height)
  {
    m_Height = height;
  }

  /**
   * Assignment operator
   */
  inline Size2 & operator=(const Size2 & rOther)
  {
    m_Width = rOther.m_Width;
    m_Height = rOther.m_Height;

    return *this;
  }

  /**
   * Equality operator
   */
  inline kt_bool operator==(const Size2 & rOther) const
  {
    return m_Width == rOther.m_Width && m_Height == rOther.m_Height;
  }

  /**
   * Inequality operator
   */
  inline kt_bool operator!=(const Size2 & rOther) const
  {
    return m_Width != rOther.m_Width || m_Height != rOther.m_Height;
  }

  /**
   * Write Size2 onto output stream
   * @param rStream output stream
   * @param rSize to write
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Size2 & rSize)
  {
    rStream << "(" << rSize.m_Width << ", " << rSize.m_Height << ")";
    return rStream;
  }

private:
  T m_Width;
  T m_Height;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Width);
    ar & BOOST_SERIALIZATION_NVP(m_Height);
  }
};    // Size2<T>

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Represents a vector (x, y) in 2-dimensional real space.
 */
template<typename T>
class Vector2
{
public:
  /**
   * Default constructor
   */
  Vector2()
  {
    m_Values[0] = 0;
    m_Values[1] = 0;
  }

  /**
   * Constructor initializing vector location
   * @param x
   * @param y
   */
  Vector2(T x, T y)
  {
    m_Values[0] = x;
    m_Values[1] = y;
  }

public:
  /**
   * Gets the x-coordinate of this vector2
   * @return the x-coordinate of the vector2
   */
  inline const T & GetX() const
  {
    return m_Values[0];
  }

  /**
   * Sets the x-coordinate of this vector2
   * @param x the x-coordinate of the vector2
   */
  inline void SetX(const T & x)
  {
    m_Values[0] = x;
  }

  /**
   * Gets the y-coordinate of this vector2
   * @return the y-coordinate of the vector2
   */
  inline const T & GetY() const
  {
    return m_Values[1];
  }

  /**
   * Sets the y-coordinate of this vector2
   * @param y the y-coordinate of the vector2
   */
  inline void SetY(const T & y)
  {
    m_Values[1] = y;
  }

  /**
   * Floor point operator
   * @param rOther
   */
  inline void MakeFloor(const Vector2 & rOther)
  {
    if (rOther.m_Values[0] < m_Values[0]) {m_Values[0] = rOther.m_Values[0];}
    if (rOther.m_Values[1] < m_Values[1]) {m_Values[1] = rOther.m_Values[1];}
  }

  /**
   * Ceiling point operator
   * @param rOther
   */
  inline void MakeCeil(const Vector2 & rOther)
  {
    if (rOther.m_Values[0] > m_Values[0]) {m_Values[0] = rOther.m_Values[0];}
    if (rOther.m_Values[1] > m_Values[1]) {m_Values[1] = rOther.m_Values[1];}
  }

  /**
   * Returns the square of the length of the vector
   * @return square of the length of the vector
   */
  inline kt_double SquaredLength() const
  {
    return math::Square(m_Values[0]) + math::Square(m_Values[1]);
  }

  /**
   * Returns the length of the vector (x and y).
   * @return length of the vector
   */
  inline kt_double Length() const
  {
    return sqrt(SquaredLength());
  }

  /**
   * Returns the square distance to the given vector
   * @returns square distance to the given vector
   */
  inline kt_double SquaredDistance(const Vector2 & rOther) const
  {
    return (*this - rOther).SquaredLength();
  }

  /**
   * Gets the distance to the other vector2
   * @param rOther
   * @return distance to other vector2
   */
  inline kt_double Distance(const Vector2 & rOther) const
  {
    return sqrt(SquaredDistance(rOther));
  }

public:
  /**
   * In place Vector2 addition.
   */
  inline void operator+=(const Vector2 & rOther)
  {
    m_Values[0] += rOther.m_Values[0];
    m_Values[1] += rOther.m_Values[1];
  }

  /**
   * In place Vector2 subtraction.
   */
  inline void operator-=(const Vector2 & rOther)
  {
    m_Values[0] -= rOther.m_Values[0];
    m_Values[1] -= rOther.m_Values[1];
  }

  /**
   * Addition operator
   * @param rOther
   * @return vector resulting from adding this vector with the given vector
   */
  inline const Vector2 operator+(const Vector2 & rOther) const
  {
    return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
  }

  /**
   * Subtraction operator
   * @param rOther
   * @return vector resulting from subtracting this vector from the given vector
   */
  inline const Vector2 operator-(const Vector2 & rOther) const
  {
    return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
  }

  /**
   * In place scalar division operator
   * @param scalar
   */
  inline void operator/=(T scalar)
  {
    m_Values[0] /= scalar;
    m_Values[1] /= scalar;
  }

  /**
   * Divides a Vector2
   * @param scalar
   * @return scalar product
   */
  inline const Vector2 operator/(T scalar) const
  {
    return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
  }

  /**
   * Computes the dot product between the two vectors
   * @param rOther
   * @return dot product
   */
  inline kt_double operator*(const Vector2 & rOther) const
  {
    return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
  }

  /**
   * Scales the vector by the given scalar
   * @param scalar
   */
  inline const Vector2 operator*(T scalar) const
  {
    return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
  }

  /**
   * Subtract the vector by the given scalar
   * @param scalar
   */
  inline const Vector2 operator-(T scalar) const
  {
    return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
  }

  /**
   * In place scalar multiplication operator
   * @param scalar
   */
  inline void operator*=(T scalar)
  {
    m_Values[0] *= scalar;
    m_Values[1] *= scalar;
  }

  /**
   * Equality operator returns true if the corresponding x, y values of each Vector2 are the same values.
   * @param rOther
   */
  inline kt_bool operator==(const Vector2 & rOther) const
  {
    return m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1];
  }

  /**
   * Inequality operator returns true if any of the corresponding x, y values of each Vector2 not the same.
   * @param rOther
   */
  inline kt_bool operator!=(const Vector2 & rOther) const
  {
    return m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1];
  }

  /**
   * Less than operator
   * @param rOther
   * @return true if left vector is less than right vector
   */
  inline kt_bool operator<(const Vector2 & rOther) const
  {
    if (m_Values[0] < rOther.m_Values[0]) {
      return true;
    } else if (m_Values[0] > rOther.m_Values[0]) {
      return false;
    } else {
      return m_Values[1] < rOther.m_Values[1];
    }
  }

  /**
   * Write Vector2 onto output stream
   * @param rStream output stream
   * @param rVector to write
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Vector2 & rVector)
  {
    rStream << rVector.GetX() << " " << rVector.GetY();
    return rStream;
  }

  /**
   * Read Vector2 from input stream
   * @param rStream input stream
   */
  friend inline std::istream & operator>>(std::istream & rStream, const Vector2 & /*rVector*/)
  {
    // Implement me!!  TODO(lucbettaieb): What the what?  Do I need to implement this?
    return rStream;
  }

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & boost::serialization::make_nvp("m_Values_0", m_Values[0]);
    ar & boost::serialization::make_nvp("m_Values_1", m_Values[1]);
  }

private:
  T m_Values[2];
};    // Vector2<T>

/**
 * Type declaration of Vector2<kt_double> vector
 */
typedef std::vector<Vector2<kt_double>> PointVectorDouble;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Represents a vector (x, y, z) in 3-dimensional real space.
 */
template<typename T>
class Vector3
{
public:
  /**
   * Default constructor
   */
  Vector3()
  {
    m_Values[0] = 0;
    m_Values[1] = 0;
    m_Values[2] = 0;
  }

  /**
   * Constructor initializing point location
   * @param x
   * @param y
   * @param z
   */
  Vector3(T x, T y, T z)
  {
    m_Values[0] = x;
    m_Values[1] = y;
    m_Values[2] = z;
  }

  /**
   * Copy constructor
   * @param rOther
   */
  Vector3(const Vector3 & rOther)
  {
    m_Values[0] = rOther.m_Values[0];
    m_Values[1] = rOther.m_Values[1];
    m_Values[2] = rOther.m_Values[2];
  }

public:
  /**
   * Gets the x-component of this vector
   * @return x-component
   */
  inline const T & GetX() const
  {
    return m_Values[0];
  }

  /**
   * Sets the x-component of this vector
   * @param x
   */
  inline void SetX(const T & x)
  {
    m_Values[0] = x;
  }

  /**
   * Gets the y-component of this vector
   * @return y-component
   */
  inline const T & GetY() const
  {
    return m_Values[1];
  }

  /**
   * Sets the y-component of this vector
   * @param y
   */
  inline void SetY(const T & y)
  {
    m_Values[1] = y;
  }

  /**
   * Gets the z-component of this vector
   * @return z-component
   */
  inline const T & GetZ() const
  {
    return m_Values[2];
  }

  /**
   * Sets the z-component of this vector
   * @param z
   */
  inline void SetZ(const T & z)
  {
    m_Values[2] = z;
  }

  /**
   * Floor vector operator
   * @param rOther Vector3d
   */
  inline void MakeFloor(const Vector3 & rOther)
  {
    if (rOther.m_Values[0] < m_Values[0]) {m_Values[0] = rOther.m_Values[0];}
    if (rOther.m_Values[1] < m_Values[1]) {m_Values[1] = rOther.m_Values[1];}
    if (rOther.m_Values[2] < m_Values[2]) {m_Values[2] = rOther.m_Values[2];}
  }

  /**
   * Ceiling vector operator
   * @param rOther Vector3d
   */
  inline void MakeCeil(const Vector3 & rOther)
  {
    if (rOther.m_Values[0] > m_Values[0]) {m_Values[0] = rOther.m_Values[0];}
    if (rOther.m_Values[1] > m_Values[1]) {m_Values[1] = rOther.m_Values[1];}
    if (rOther.m_Values[2] > m_Values[2]) {m_Values[2] = rOther.m_Values[2];}
  }

  /**
   * Returns the square of the length of the vector
   * @return square of the length of the vector
   */
  inline kt_double SquaredLength() const
  {
    return math::Square(m_Values[0]) + math::Square(m_Values[1]) + math::Square(m_Values[2]);
  }

  /**
   * Returns the length of the vector.
   * @return Length of the vector
   */
  inline kt_double Length() const
  {
    return sqrt(SquaredLength());
  }

  /**
   * Returns a string representation of this vector
   * @return string representation of this vector
   */
  inline std::string ToString() const
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);

    converter << GetX() << " " << GetY() << " " << GetZ();

    return converter.str();
  }

public:
  /**
   * Assignment operator
   */
  inline Vector3 & operator=(const Vector3 & rOther)
  {
    m_Values[0] = rOther.m_Values[0];
    m_Values[1] = rOther.m_Values[1];
    m_Values[2] = rOther.m_Values[2];

    return *this;
  }

  /**
   * Binary vector add.
   * @param rOther
   * @return vector sum
   */
  inline const Vector3 operator+(const Vector3 & rOther) const
  {
    return Vector3(m_Values[0] + rOther.m_Values[0],
             m_Values[1] + rOther.m_Values[1],
             m_Values[2] + rOther.m_Values[2]);
  }

  /**
   * Binary vector add.
   * @param scalar
   * @return sum
   */
  inline const Vector3 operator+(kt_double scalar) const
  {
    return Vector3(m_Values[0] + scalar,
             m_Values[1] + scalar,
             m_Values[2] + scalar);
  }

  /**
   * Binary vector subtract.
   * @param rOther
   * @return vector difference
   */
  inline const Vector3 operator-(const Vector3 & rOther) const
  {
    return Vector3(m_Values[0] - rOther.m_Values[0],
             m_Values[1] - rOther.m_Values[1],
             m_Values[2] - rOther.m_Values[2]);
  }

  /**
   * Binary vector subtract.
   * @param scalar
   * @return difference
   */
  inline const Vector3 operator-(kt_double scalar) const
  {
    return Vector3(m_Values[0] - scalar, m_Values[1] - scalar, m_Values[2] - scalar);
  }

  /**
   * Scales the vector by the given scalar
   * @param scalar
   */
  inline const Vector3 operator*(T scalar) const
  {
    return Vector3(m_Values[0] * scalar, m_Values[1] * scalar, m_Values[2] * scalar);
  }

  /**
   * Equality operator returns true if the corresponding x, y, z values of each Vector3 are the same values.
   * @param rOther
   */
  inline kt_bool operator==(const Vector3 & rOther) const
  {
    return m_Values[0] == rOther.m_Values[0] &&
           m_Values[1] == rOther.m_Values[1] &&
           m_Values[2] == rOther.m_Values[2];
  }

  /**
   * Inequality operator returns true if any of the corresponding x, y, z values of each Vector3 not the same.
   * @param rOther
   */
  inline kt_bool operator!=(const Vector3 & rOther) const
  {
    return m_Values[0] != rOther.m_Values[0] ||
           m_Values[1] != rOther.m_Values[1] ||
           m_Values[2] != rOther.m_Values[2];
  }

  /**
   * Write Vector3 onto output stream
   * @param rStream output stream
   * @param rVector to write
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Vector3 & rVector)
  {
    rStream << rVector.ToString();
    return rStream;
  }

  /**
   * Read Vector3 from input stream
   * @param rStream input stream
   */
  friend inline std::istream & operator>>(std::istream & rStream, const Vector3 & /*rVector*/)
  {
    // Implement me!!
    return rStream;
  }

private:
  T m_Values[3];
};    // Vector3

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines an orientation as a quaternion rotation using the positive Z axis as the zero reference.
 * <BR>
 * Q = w + ix + jy + kz <BR>
 * w = c_1 * c_2 * c_3 - s_1 * s_2 * s_3 <BR>
 * x = s_1 * s_2 * c_3 + c_1 * c_2 * s_3 <BR>
 * y = s_1 * c_2 * c_3 + c_1 * s_2 * s_3 <BR>
 * z = c_1 * s_2 * c_3 - s_1 * c_2 * s_3 <BR>
 * where <BR>
 * c_1 = cos(theta/2) <BR>
 * c_2 = cos(phi/2) <BR>
 * c_3 = cos(psi/2) <BR>
 * s_1 = sin(theta/2) <BR>
 * s_2 = sin(phi/2) <BR>
 * s_3 = sin(psi/2) <BR>
 * and <BR>
 * theta is the angle of rotation about the Y axis measured from the Z axis. <BR>
 * phi is the angle of rotation about the Z axis measured from the X axis. <BR>
 * psi is the angle of rotation about the X axis measured from the Y axis. <BR>
 * (All angles are right-handed.)
 */
class Quaternion
{
public:
  /**
   * Create a quaternion with default (x=0, y=0, z=0, w=1) values
   */
  inline Quaternion()
  {
    m_Values[0] = 0.0;
    m_Values[1] = 0.0;
    m_Values[2] = 0.0;
    m_Values[3] = 1.0;
  }

  /**
   * Create a quaternion using x, y, z, w values.
   * @param x
   * @param y
   * @param z
   * @param w
   */
  inline Quaternion(kt_double x, kt_double y, kt_double z, kt_double w)
  {
    m_Values[0] = x;
    m_Values[1] = y;
    m_Values[2] = z;
    m_Values[3] = w;
  }

  /**
   * Copy constructor
   */
  inline Quaternion(const Quaternion & rQuaternion)
  {
    m_Values[0] = rQuaternion.m_Values[0];
    m_Values[1] = rQuaternion.m_Values[1];
    m_Values[2] = rQuaternion.m_Values[2];
    m_Values[3] = rQuaternion.m_Values[3];
  }

public:
  /**
   * Returns the X-value
   * @return Return the X-value of the quaternion
   */
  inline kt_double GetX() const
  {
    return m_Values[0];
  }

  /**
   * Sets the X-value
   * @param x X-value of the quaternion
   */
  inline void SetX(kt_double x)
  {
    m_Values[0] = x;
  }

  /**
   * Returns the Y-value
   * @return Return the Y-value of the quaternion
   */
  inline kt_double GetY() const
  {
    return m_Values[1];
  }

  /**
   * Sets the Y-value
   * @param y Y-value of the quaternion
   */
  inline void SetY(kt_double y)
  {
    m_Values[1] = y;
  }

  /**
   * Returns the Z-value
   * @return Return the Z-value of the quaternion
   */
  inline kt_double GetZ() const
  {
    return m_Values[2];
  }

  /**
   * Sets the Z-value
   * @param z Z-value of the quaternion
   */
  inline void SetZ(kt_double z)
  {
    m_Values[2] = z;
  }

  /**
   * Returns the W-value
   * @return Return the W-value of the quaternion
   */
  inline kt_double GetW() const
  {
    return m_Values[3];
  }

  /**
   * Sets the W-value
   * @param w W-value of the quaternion
   */
  inline void SetW(kt_double w)
  {
    m_Values[3] = w;
  }

  /**
   * Converts this quaternion into Euler angles
   * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
   * @param rYaw
   * @param rPitch
   * @param rRoll
   */
  void ToEulerAngles(kt_double & rYaw, kt_double & rPitch, kt_double & rRoll) const
  {
    kt_double test = m_Values[0] * m_Values[1] + m_Values[2] * m_Values[3];

    if (test > 0.499) {
      // singularity at north pole
      rYaw = 2 * atan2(m_Values[0], m_Values[3]);
      rPitch = KT_PI_2;
      rRoll = 0;
    } else if (test < -0.499) {
      // singularity at south pole
      rYaw = -2 * atan2(m_Values[0], m_Values[3]);
      rPitch = -KT_PI_2;
      rRoll = 0;
    } else {
      kt_double sqx = m_Values[0] * m_Values[0];
      kt_double sqy = m_Values[1] * m_Values[1];
      kt_double sqz = m_Values[2] * m_Values[2];

      rYaw = atan2(2 * m_Values[1] * m_Values[3] - 2 * m_Values[0] * m_Values[2],
          1 - 2 * sqy - 2 * sqz);
      rPitch = asin(2 * test);
      rRoll = atan2(2 * m_Values[0] * m_Values[3] - 2 * m_Values[1] * m_Values[2],
          1 - 2 * sqx - 2 * sqz);
    }
  }

  /**
   * Set x,y,z,w values of the quaternion based on Euler angles.
   * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
   * @param yaw
   * @param pitch
   * @param roll
   */
  void FromEulerAngles(kt_double yaw, kt_double pitch, kt_double roll)
  {
    kt_double angle;

    angle = yaw * 0.5;
    kt_double cYaw = cos(angle);
    kt_double sYaw = sin(angle);

    angle = pitch * 0.5;
    kt_double cPitch = cos(angle);
    kt_double sPitch = sin(angle);

    angle = roll * 0.5;
    kt_double cRoll = cos(angle);
    kt_double sRoll = sin(angle);

    m_Values[0] = sYaw * sPitch * cRoll + cYaw * cPitch * sRoll;
    m_Values[1] = sYaw * cPitch * cRoll + cYaw * sPitch * sRoll;
    m_Values[2] = cYaw * sPitch * cRoll - sYaw * cPitch * sRoll;
    m_Values[3] = cYaw * cPitch * cRoll - sYaw * sPitch * sRoll;
  }

  /**
   * Assignment operator
   * @param rQuaternion
   */
  inline Quaternion & operator=(const Quaternion & rQuaternion)
  {
    m_Values[0] = rQuaternion.m_Values[0];
    m_Values[1] = rQuaternion.m_Values[1];
    m_Values[2] = rQuaternion.m_Values[2];
    m_Values[3] = rQuaternion.m_Values[3];

    return *this;
  }

  /**
   * Equality operator returns true if the corresponding x, y, z, w values of each quaternion are the same values.
   * @param rOther
   */
  inline kt_bool operator==(const Quaternion & rOther) const
  {
    return m_Values[0] == rOther.m_Values[0] &&
           m_Values[1] == rOther.m_Values[1] &&
           m_Values[2] == rOther.m_Values[2] &&
           m_Values[3] == rOther.m_Values[3];
  }

  /**
   * Inequality operator returns true if any of the corresponding x, y, z, w values of each quaternion not the same.
   * @param rOther
   */
  inline kt_bool operator!=(const Quaternion & rOther) const
  {
    return m_Values[0] != rOther.m_Values[0] ||
           m_Values[1] != rOther.m_Values[1] ||
           m_Values[2] != rOther.m_Values[2] ||
           m_Values[3] != rOther.m_Values[3];
  }

  /**
   * Write this quaternion onto output stream
   * @param rStream output stream
   * @param rQuaternion
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Quaternion & rQuaternion)
  {
    rStream << rQuaternion.m_Values[0] << " " <<
      rQuaternion.m_Values[1] << " " <<
      rQuaternion.m_Values[2] << " " <<
      rQuaternion.m_Values[3];
    return rStream;
  }

private:
  kt_double m_Values[4];
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Stores x, y, width and height that represents the location and size of a rectangle
 * (x, y) is at bottom left in mapper!
 */
template<typename T>
class Rectangle2
{
public:
  /**
   * Default constructor
   */
  Rectangle2()
  {
  }

  /**
   * Constructor initializing rectangle parameters
   * @param x x-coordinate of left edge of rectangle
   * @param y y-coordinate of bottom edge of rectangle
   * @param width width of rectangle
   * @param height height of rectangle
   */
  Rectangle2(T x, T y, T width, T height)
  : m_Position(x, y),
    m_Size(width, height)
  {
  }

  /**
   * Constructor initializing rectangle parameters
   * @param rPosition (x,y)-coordinate of rectangle
   * @param rSize Size of the rectangle
   */
  Rectangle2(const Vector2<T> & rPosition, const Size2<T> & rSize)
  : m_Position(rPosition),
    m_Size(rSize)
  {
  }

  /**
   * Copy constructor
   */
  Rectangle2(const Rectangle2 & rOther)
  : m_Position(rOther.m_Position),
    m_Size(rOther.m_Size)
  {
  }

public:
  /**
   * Gets the x-coordinate of the left edge of this rectangle
   * @return the x-coordinate of the left edge of this rectangle
   */
  inline T GetX() const
  {
    return m_Position.GetX();
  }

  /**
   * Sets the x-coordinate of the left edge of this rectangle
   * @param x the x-coordinate of the left edge of this rectangle
   */
  inline void SetX(T x)
  {
    m_Position.SetX(x);
  }

  /**
   * Gets the y-coordinate of the bottom edge of this rectangle
   * @return the y-coordinate of the bottom edge of this rectangle
   */
  inline T GetY() const
  {
    return m_Position.GetY();
  }

  /**
   * Sets the y-coordinate of the bottom edge of this rectangle
   * @param y the y-coordinate of the bottom edge of this rectangle
   */
  inline void SetY(T y)
  {
    m_Position.SetY(y);
  }

  /**
   * Gets the width of this rectangle
   * @return the width of this rectangle
   */
  inline T GetWidth() const
  {
    return m_Size.GetWidth();
  }

  /**
   * Sets the width of this rectangle
   * @param width the width of this rectangle
   */
  inline void SetWidth(T width)
  {
    m_Size.SetWidth(width);
  }

  /**
   * Gets the height of this rectangle
   * @return the height of this rectangle
   */
  inline T GetHeight() const
  {
    return m_Size.GetHeight();
  }

  /**
   * Sets the height of this rectangle
   * @param height the height of this rectangle
   */
  inline void SetHeight(T height)
  {
    m_Size.SetHeight(height);
  }

  /**
   * Gets the position of this rectangle
   * @return the position of this rectangle
   */
  inline const Vector2<T> & GetPosition() const
  {
    return m_Position;
  }

  /**
   * Sets the position of this rectangle
   * @param rX x
   * @param rY y
   */
  inline void SetPosition(const T & rX, const T & rY)
  {
    m_Position = Vector2<T>(rX, rY);
  }

  /**
   * Sets the position of this rectangle
   * @param rPosition position
   */
  inline void SetPosition(const Vector2<T> & rPosition)
  {
    m_Position = rPosition;
  }

  /**
   * Gets the size of this rectangle
   * @return the size of this rectangle
   */
  inline const Size2<T> & GetSize() const
  {
    return m_Size;
  }

  /**
   * Sets the size of this rectangle
   * @param rSize size
   */
  inline void SetSize(const Size2<T> & rSize)
  {
    m_Size = rSize;
  }

  /**
   * Gets the center of this rectangle
   * @return the center of this rectangle
   */
  inline const Vector2<T> GetCenter() const
  {
    return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5,
             m_Position.GetY() + m_Size.GetHeight() * 0.5);
  }

public:
  /**
   * Assignment operator
   */
  Rectangle2 & operator=(const Rectangle2 & rOther)
  {
    m_Position = rOther.m_Position;
    m_Size = rOther.m_Size;

    return *this;
  }

  /**
   * Equality operator
   */
  inline kt_bool operator==(const Rectangle2 & rOther) const
  {
    return m_Position == rOther.m_Position && m_Size == rOther.m_Size;
  }

  /**
   * Inequality operator
   */
  inline kt_bool operator!=(const Rectangle2 & rOther) const
  {
    return m_Position != rOther.m_Position || m_Size != rOther.m_Size;
  }

private:
  Vector2<T> m_Position;
  Size2<T> m_Size;
  /**
   * Serialization: class Rectangle2
   */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Position);
    ar & BOOST_SERIALIZATION_NVP(m_Size);
  }
};    // Rectangle2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class Pose3;

/**
 * Defines a position (x, y) in 2-dimensional space and heading.
 */
class Pose2
{
public:
  /**
   * Default Constructor
   */
  Pose2()
  : m_Heading(0.0)
  {
  }

  /**
   * Constructor initializing pose parameters
   * @param rPosition position
   * @param heading heading
   **/
  Pose2(const Vector2<kt_double> & rPosition, kt_double heading)
  : m_Position(rPosition),
    m_Heading(heading)
  {
  }

  /**
   * Constructor initializing pose parameters
   * @param x x-coordinate
   * @param y y-coordinate
   * @param heading heading
   **/
  Pose2(kt_double x, kt_double y, kt_double heading)
  : m_Position(x, y),
    m_Heading(heading)
  {
  }

  /**
   * Constructs a Pose2 object from a Pose3.
   */
  Pose2(const Pose3 & rPose);  // NOLINT

  /**
   * Copy constructor
   */
  Pose2(const Pose2 & rOther)
  : m_Position(rOther.m_Position),
    m_Heading(rOther.m_Heading)
  {
  }

public:
  /**
   * Returns the x-coordinate
   * @return the x-coordinate of the pose
   */
  inline kt_double GetX() const
  {
    return m_Position.GetX();
  }

  /**
   * Sets the x-coordinate
   * @param x the x-coordinate of the pose
   */
  inline void SetX(kt_double x)
  {
    m_Position.SetX(x);
  }

  /**
   * Returns the y-coordinate
   * @return the y-coordinate of the pose
   */
  inline kt_double GetY() const
  {
    return m_Position.GetY();
  }

  /**
   * Sets the y-coordinate
   * @param y the y-coordinate of the pose
   */
  inline void SetY(kt_double y)
  {
    m_Position.SetY(y);
  }

  /**
   * Returns the position
   * @return the position of the pose
   */
  inline const Vector2<kt_double> & GetPosition() const
  {
    return m_Position;
  }

  /**
   * Sets the position
   * @param rPosition of the pose
   */
  inline void SetPosition(const Vector2<kt_double> & rPosition)
  {
    m_Position = rPosition;
  }

  /**
   * Returns the heading of the pose (in radians)
   * @return the heading of the pose
   */
  inline kt_double GetHeading() const
  {
    return m_Heading;
  }

  /**
   * Sets the heading
   * @param heading of the pose
   */
  inline void SetHeading(kt_double heading)
  {
    m_Heading = heading;
  }

  /**
   * Return the squared distance between two Pose2
   * @return squared distance
   */
  inline kt_double SquaredDistance(const Pose2 & rOther) const
  {
    return m_Position.SquaredDistance(rOther.m_Position);
  }

public:
  /**
   * Assignment operator
   */
  inline Pose2 & operator=(const Pose2 & rOther)
  {
    m_Position = rOther.m_Position;
    m_Heading = rOther.m_Heading;

    return *this;
  }

  /**
   * Equality operator
   */
  inline kt_bool operator==(const Pose2 & rOther) const
  {
    return m_Position == rOther.m_Position && m_Heading == rOther.m_Heading;
  }

  /**
   * Inequality operator
   */
  inline kt_bool operator!=(const Pose2 & rOther) const
  {
    return m_Position != rOther.m_Position || m_Heading != rOther.m_Heading;
  }

  /**
   * In place Pose2 add.
   */
  inline void operator+=(const Pose2 & rOther)
  {
    m_Position += rOther.m_Position;
    m_Heading = math::NormalizeAngle(m_Heading + rOther.m_Heading);
  }

  /**
   * Binary Pose2 add
   * @param rOther
   * @return Pose2 sum
   */
  inline Pose2 operator+(const Pose2 & rOther) const
  {
    return Pose2(m_Position + rOther.m_Position,
             math::NormalizeAngle(m_Heading + rOther.m_Heading));
  }

  /**
   * Binary Pose2 subtract
   * @param rOther
   * @return Pose2 difference
   */
  inline Pose2 operator-(const Pose2 & rOther) const
  {
    return Pose2(m_Position - rOther.m_Position,
             math::NormalizeAngle(m_Heading - rOther.m_Heading));
  }

  /**
   * Read pose from input stream
   * @param rStream input stream
   */
  friend inline std::istream & operator>>(std::istream & rStream, const Pose2 & /*rPose*/)
  {
    // Implement me!!
    return rStream;
  }

  /**
   * Write this pose onto output stream
   * @param rStream output stream
   * @param rPose to read
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Pose2 & rPose)
  {
    rStream << rPose.m_Position.GetX() << " " << rPose.m_Position.GetY() << " " << rPose.m_Heading;
    return rStream;
  }

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Position);
    ar & BOOST_SERIALIZATION_NVP(m_Heading);
  }

private:
  Vector2<kt_double> m_Position;

  kt_double m_Heading;
};    // Pose2

/**
 * Type declaration of Pose2 vector
 */
typedef std::vector<Pose2> Pose2Vector;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a position and orientation in 3-dimensional space.
 * Karto uses a right-handed coordinate system with X, Y as the 2-D ground plane and X is forward and Y is left.
 * Values in Vector3 used to define position must have units of meters.
 * The value of angle when defining orientation in two dimensions must be in units of radians.
 * The definition of orientation in three dimensions uses quaternions.
 */
class Pose3
{
public:
  /**
   * Default constructor
   */
  Pose3()
  {
  }

  /**
   * Create a new Pose3 object from the given position.
   * @param rPosition position vector in three space.
   */
  Pose3(const Vector3<kt_double> & rPosition)  // NOLINT
  : m_Position(rPosition)
  {
  }

  /**
   * Create a new Pose3 object from the given position and orientation.
   * @param rPosition position vector in three space.
   * @param rOrientation quaternion orientation in three space.
   */
  Pose3(const Vector3<kt_double> & rPosition, const karto::Quaternion & rOrientation)
  : m_Position(rPosition),
    m_Orientation(rOrientation)
  {
  }

  /**
   * Copy constructor
   */
  Pose3(const Pose3 & rOther)
  : m_Position(rOther.m_Position),
    m_Orientation(rOther.m_Orientation)
  {
  }

  /**
   * Constructs a Pose3 object from a Pose2.
   */
  Pose3(const Pose2 & rPose)  // NOLINT
  {
    m_Position = Vector3<kt_double>(rPose.GetX(), rPose.GetY(), 0.0);
    m_Orientation.FromEulerAngles(rPose.GetHeading(), 0.0, 0.0);
  }

public:
  /**
   * Get the position of the pose as a 3D vector as const. Values have units of meters.
   * @return 3-dimensional position vector as const
   */
  inline const Vector3<kt_double> & GetPosition() const
  {
    return m_Position;
  }

  /**
   * Set the position of the pose as a 3D vector. Values have units of meters.
   * @return 3-dimensional position vector
   */
  inline void SetPosition(const Vector3<kt_double> & rPosition)
  {
    m_Position = rPosition;
  }

  /**
   * Get the orientation quaternion of the pose as const.
   * @return orientation quaternion as const
   */
  inline const Quaternion & GetOrientation() const
  {
    return m_Orientation;
  }

  /**
   * Get the orientation quaternion of the pose.
   * @return orientation quaternion
   */
  inline void SetOrientation(const Quaternion & rOrientation)
  {
    m_Orientation = rOrientation;
  }

  /**
   * Returns a string representation of this pose
   * @return string representation of this pose
   */
  inline std::string ToString()
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);

    converter << GetPosition() << " " << GetOrientation();

    return converter.str();
  }

public:
  /**
   * Assignment operator
   */
  inline Pose3 & operator=(const Pose3 & rOther)
  {
    m_Position = rOther.m_Position;
    m_Orientation = rOther.m_Orientation;

    return *this;
  }

  /**
   * Equality operator
   */
  inline kt_bool operator==(const Pose3 & rOther) const
  {
    return m_Position == rOther.m_Position && m_Orientation == rOther.m_Orientation;
  }

  /**
   * Inequality operator
   */
  inline kt_bool operator!=(const Pose3 & rOther) const
  {
    return m_Position != rOther.m_Position || m_Orientation != rOther.m_Orientation;
  }

  /**
   * Write Pose3 onto output stream
   * @param rStream output stream
   * @param rPose to write
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Pose3 & rPose)
  {
    rStream << rPose.GetPosition() << ", " << rPose.GetOrientation();
    return rStream;
  }

  /**
   * Read Pose3 from input stream
   * @param rStream input stream
   */
  friend inline std::istream & operator>>(std::istream & rStream, const Pose3 & /*rPose*/)
  {
    // Implement me!!
    return rStream;
  }

private:
  Vector3<kt_double> m_Position;
  Quaternion m_Orientation;
};    // Pose3

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a Matrix 3 x 3 class.
 */
class Matrix3
{
public:
  /**
   * Default constructor
   */
  Matrix3()
  {
    Clear();
  }

  /**
   * Copy constructor
   */
  inline Matrix3(const Matrix3 & rOther)
  {
    memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(kt_double));
  }

public:
  /**
   * Sets this matrix to identity matrix
   */
  void SetToIdentity()
  {
    memset(m_Matrix, 0, 9 * sizeof(kt_double));

    for (kt_int32s i = 0; i < 3; i++) {
      m_Matrix[i][i] = 1.0;
    }
  }

  /**
   * Sets this matrix to zero matrix
   */
  void Clear()
  {
    memset(m_Matrix, 0, 9 * sizeof(kt_double));
  }

  /**
   * Sets this matrix to be the rotation matrix of rotation around given axis
   * @param x x-coordinate of axis
   * @param y y-coordinate of axis
   * @param z z-coordinate of axis
   * @param radians amount of rotation
   */
  void FromAxisAngle(kt_double x, kt_double y, kt_double z, const kt_double radians)
  {
    kt_double cosRadians = cos(radians);
    kt_double sinRadians = sin(radians);
    kt_double oneMinusCos = 1.0 - cosRadians;

    kt_double xx = x * x;
    kt_double yy = y * y;
    kt_double zz = z * z;

    kt_double xyMCos = x * y * oneMinusCos;
    kt_double xzMCos = x * z * oneMinusCos;
    kt_double yzMCos = y * z * oneMinusCos;

    kt_double xSin = x * sinRadians;
    kt_double ySin = y * sinRadians;
    kt_double zSin = z * sinRadians;

    m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
    m_Matrix[0][1] = xyMCos - zSin;
    m_Matrix[0][2] = xzMCos + ySin;

    m_Matrix[1][0] = xyMCos + zSin;
    m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
    m_Matrix[1][2] = yzMCos - xSin;

    m_Matrix[2][0] = xzMCos - ySin;
    m_Matrix[2][1] = yzMCos + xSin;
    m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
  }

  /**
   * Returns transposed version of this matrix
   * @return transposed matrix
   */
  Matrix3 Transpose() const
  {
    Matrix3 transpose;

    for (kt_int32u row = 0; row < 3; row++) {
      for (kt_int32u col = 0; col < 3; col++) {
        transpose.m_Matrix[row][col] = m_Matrix[col][row];
      }
    }

    return transpose;
  }

  /**
   * Returns the inverse of the matrix
   */
  Matrix3 Inverse() const
  {
    Matrix3 kInverse = *this;
    kt_bool haveInverse = InverseFast(kInverse, 1e-14);
    if (haveInverse == false) {
      assert(false);
    }
    return kInverse;
  }

  /**
   * Internal helper method for inverse matrix calculation
   * This code is lifted from the OgreMatrix3 class!!
   */
  kt_bool InverseFast(Matrix3 & rkInverse, kt_double fTolerance = KT_TOLERANCE) const
  {
    // Invert a 3x3 using cofactors.  This is about 8 times faster than
    // the Numerical Recipes code which uses Gaussian elimination.
    rkInverse.m_Matrix[0][0] = m_Matrix[1][1] * m_Matrix[2][2] - m_Matrix[1][2] * m_Matrix[2][1];
    rkInverse.m_Matrix[0][1] = m_Matrix[0][2] * m_Matrix[2][1] - m_Matrix[0][1] * m_Matrix[2][2];
    rkInverse.m_Matrix[0][2] = m_Matrix[0][1] * m_Matrix[1][2] - m_Matrix[0][2] * m_Matrix[1][1];
    rkInverse.m_Matrix[1][0] = m_Matrix[1][2] * m_Matrix[2][0] - m_Matrix[1][0] * m_Matrix[2][2];
    rkInverse.m_Matrix[1][1] = m_Matrix[0][0] * m_Matrix[2][2] - m_Matrix[0][2] * m_Matrix[2][0];
    rkInverse.m_Matrix[1][2] = m_Matrix[0][2] * m_Matrix[1][0] - m_Matrix[0][0] * m_Matrix[1][2];
    rkInverse.m_Matrix[2][0] = m_Matrix[1][0] * m_Matrix[2][1] - m_Matrix[1][1] * m_Matrix[2][0];
    rkInverse.m_Matrix[2][1] = m_Matrix[0][1] * m_Matrix[2][0] - m_Matrix[0][0] * m_Matrix[2][1];
    rkInverse.m_Matrix[2][2] = m_Matrix[0][0] * m_Matrix[1][1] - m_Matrix[0][1] * m_Matrix[1][0];

    kt_double fDet = m_Matrix[0][0] * rkInverse.m_Matrix[0][0] +
      m_Matrix[0][1] * rkInverse.m_Matrix[1][0] +
      m_Matrix[0][2] * rkInverse.m_Matrix[2][0];

    if (fabs(fDet) <= fTolerance) {
      return false;
    }

    kt_double fInvDet = 1.0 / fDet;
    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        rkInverse.m_Matrix[row][col] *= fInvDet;
      }
    }

    return true;
  }

  /**
   * Returns a string representation of this matrix
   * @return string representation of this matrix
   */
  inline std::string ToString() const
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);

    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        converter << m_Matrix[row][col] << " ";
      }
    }

    return converter.str();
  }

public:
  /**
   * Assignment operator
   */
  inline Matrix3 & operator=(const Matrix3 & rOther)
  {
    memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(kt_double));
    return *this;
  }

  /**
   * Matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return reference to mat(r,c)
   */
  inline kt_double & operator()(kt_int32u row, kt_int32u column)
  {
    return m_Matrix[row][column];
  }

  /**
   * Read-only matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return mat(r,c)
   */
  inline kt_double operator()(kt_int32u row, kt_int32u column) const
  {
    return m_Matrix[row][column];
  }

  /**
   * Binary Matrix3 multiplication.
   * @param rOther
   * @return Matrix3 product
   */
  Matrix3 operator*(const Matrix3 & rOther) const
  {
    Matrix3 product;

    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        product.m_Matrix[row][col] = m_Matrix[row][0] * rOther.m_Matrix[0][col] +
          m_Matrix[row][1] * rOther.m_Matrix[1][col] +
          m_Matrix[row][2] * rOther.m_Matrix[2][col];
      }
    }

    return product;
  }

  /**
   * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
   * @param rPose2
   * @return Pose2 product
   */
  inline Pose2 operator*(const Pose2 & rPose2) const
  {
    Pose2 pose2;

    pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] *
      rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
    pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] *
      rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
    pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] *
      rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

    return pose2;
  }

  /**
   * In place Matrix3 add.
   * @param rkMatrix
   */
  inline void operator+=(const Matrix3 & rkMatrix)
  {
    for (kt_int32u row = 0; row < 3; row++) {
      for (kt_int32u col = 0; col < 3; col++) {
        m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
      }
    }
  }

  /**
   * Write Matrix3 onto output stream
   * @param rStream output stream
   * @param rMatrix to write
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Matrix3 & rMatrix)
  {
    rStream << rMatrix.ToString();
    return rStream;
  }

private:
  kt_double m_Matrix[3][3];
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Matrix);
  }
};    // Matrix3

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a general Matrix class.
 */
class Matrix
{
public:
  /**
   * Constructs a matrix of size rows x columns
   */
  Matrix(kt_int32u rows, kt_int32u columns)
  : m_Rows(rows),
    m_Columns(columns),
    m_pData(NULL)
  {
    Allocate();

    Clear();
  }

  /**
   * Destructor
   */
  virtual ~Matrix()
  {
    delete[] m_pData;
  }

public:
  /**
   * Set all entries to 0
   */
  void Clear()
  {
    if (m_pData != NULL) {
      memset(m_pData, 0, sizeof(kt_double) * m_Rows * m_Columns);
    }
  }

  /**
   * Gets the number of rows of the matrix
   * @return nubmer of rows
   */
  inline kt_int32u GetRows() const
  {
    return m_Rows;
  }

  /**
   * Gets the number of columns of the matrix
   * @return nubmer of columns
   */
  inline kt_int32u GetColumns() const
  {
    return m_Columns;
  }

  /**
   * Returns a reference to the entry at (row,column)
   * @param row
   * @param column
   * @return reference to entry at (row,column)
   */
  inline kt_double & operator()(kt_int32u row, kt_int32u column)
  {
    RangeCheck(row, column);

    return m_pData[row + column * m_Rows];
  }

  /**
   * Returns a const reference to the entry at (row,column)
   * @param row
   * @param column
   * @return const reference to entry at (row,column)
   */
  inline const kt_double & operator()(kt_int32u row, kt_int32u column) const
  {
    RangeCheck(row, column);

    return m_pData[row + column * m_Rows];
  }

private:
  /**
   * Allocate space for the matrix
   */
  void Allocate()
  {
    try {
      if (m_pData != NULL) {
        delete[] m_pData;
      }

      m_pData = new kt_double[m_Rows * m_Columns];
    } catch (const std::bad_alloc & ex) {
      throw Exception("Matrix allocation error");
    }

    if (m_pData == NULL) {
      throw Exception("Matrix allocation error");
    }
  }

  /**
   * Checks if (row,column) is a valid entry into the matrix
   * @param row
   * @param column
   */
  inline void RangeCheck(kt_int32u row, kt_int32u column) const
  {
    if (math::IsUpTo(row, m_Rows) == false) {
      throw Exception("Matrix - RangeCheck ERROR!!!!");
    }

    if (math::IsUpTo(column, m_Columns) == false) {
      throw Exception("Matrix - RangeCheck ERROR!!!!");
    }
  }

private:
  kt_int32u m_Rows;
  kt_int32u m_Columns;

  kt_double * m_pData;
};    // Matrix

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a bounding box in 2-dimensional real space.
 */
class BoundingBox2
{
public:
  /*
   * Default constructor
   */
  BoundingBox2()
  : m_Minimum(999999999999999999.99999, 999999999999999999.99999),
    m_Maximum(-999999999999999999.99999, -999999999999999999.99999)
  {
  }

public:
  /**
   * Get bounding box minimum
   */
  inline const Vector2<kt_double> & GetMinimum() const
  {
    return m_Minimum;
  }

  /**
   * Set bounding box minimum
   */
  inline void SetMinimum(const Vector2<kt_double> & mMinimum)
  {
    m_Minimum = mMinimum;
  }

  /**
   * Get bounding box maximum
   */
  inline const Vector2<kt_double> & GetMaximum() const
  {
    return m_Maximum;
  }

  /**
   * Set bounding box maximum
   */
  inline void SetMaximum(const Vector2<kt_double> & rMaximum)
  {
    m_Maximum = rMaximum;
  }

  /**
   * Get the size of the bounding box
   */
  inline Size2<kt_double> GetSize() const
  {
    Vector2<kt_double> size = m_Maximum - m_Minimum;

    return Size2<kt_double>(size.GetX(), size.GetY());
  }

  /**
   * Add vector to bounding box
   */
  inline void Add(const Vector2<kt_double> & rPoint)
  {
    m_Minimum.MakeFloor(rPoint);
    m_Maximum.MakeCeil(rPoint);
  }

  /**
   * Add other bounding box to bounding box
   */
  inline void Add(const BoundingBox2 & rBoundingBox)
  {
    Add(rBoundingBox.GetMinimum());
    Add(rBoundingBox.GetMaximum());
  }

  /**
   * Whether the given point is in the bounds of this box
   * @param rPoint
   * @return in bounds?
   */
  inline kt_bool IsInBounds(const Vector2<kt_double> & rPoint) const
  {
    return math::InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
           math::InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY());
  }

  /**
   * Serialization: class BoundingBox2
   */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Minimum);
    ar & BOOST_SERIALIZATION_NVP(m_Maximum);
  }

private:
  Vector2<kt_double> m_Minimum;
  Vector2<kt_double> m_Maximum;
};    // BoundingBox2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Implementation of a Pose2 transform
 */
class Transform
{
public:
  /**
   * Constructs a transformation from the origin to the given pose
   * @param rPose pose
   */
  Transform(const Pose2 & rPose)  // NOLINT
  {
    SetTransform(Pose2(), rPose);
  }

  /**
   * Constructs a transformation from the first pose to the second pose
   * @param rPose1 first pose
   * @param rPose2 second pose
   */
  Transform(const Pose2 & rPose1, const Pose2 & rPose2)
  {
    SetTransform(rPose1, rPose2);
  }

public:
  /**
   * Transforms the pose according to this transform
   * @param rSourcePose pose to transform from
   * @return transformed pose
   */
  inline Pose2 TransformPose(const Pose2 & rSourcePose)
  {
    Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
    kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());

    return Pose2(newPosition.GetPosition(), angle);
  }

  /**
   * Inverse transformation of the pose according to this transform
   * @param rSourcePose pose to transform from
   * @return transformed pose
   */
  inline Pose2 InverseTransformPose(const Pose2 & rSourcePose)
  {
    Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
    kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());

    // components of transform
    return Pose2(newPosition.GetPosition(), angle);
  }

private:
  /**
   * Sets this to be the transformation from the first pose to the second pose
   * @param rPose1 first pose
   * @param rPose2 second pose
   */
  void SetTransform(const Pose2 & rPose1, const Pose2 & rPose2)
  {
    if (rPose1 == rPose2) {
      m_Rotation.SetToIdentity();
      m_InverseRotation.SetToIdentity();
      m_Transform = Pose2();
      return;
    }

    // heading transformation
    m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
    m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());

    // position transformation
    Pose2 newPosition;
    if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0) {
      newPosition = rPose2 - m_Rotation * rPose1;
    } else {
      newPosition = rPose2;
    }

    m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
  }

private:
  // pose transformation
  Pose2 m_Transform;

  Matrix3 m_Rotation;
  Matrix3 m_InverseRotation;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Transform);
    ar & BOOST_SERIALIZATION_NVP(m_Rotation);
    ar & BOOST_SERIALIZATION_NVP(m_InverseRotation);
  }
};    // Transform

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Enumerated type for valid LaserRangeFinder types
 */
typedef enum
{
  LaserRangeFinder_Custom = 0,

  LaserRangeFinder_Sick_LMS100 = 1,
  LaserRangeFinder_Sick_LMS200 = 2,
  LaserRangeFinder_Sick_LMS291 = 3,

  LaserRangeFinder_Hokuyo_UTM_30LX = 4,
  LaserRangeFinder_Hokuyo_URG_04LX = 5
} LaserRangeFinderType;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Abstract base class for Parameters
 */
class AbstractParameter
{
public:
  AbstractParameter()
  {
  }
  /**
   * Constructs a parameter with the given name
   * @param rName
   * @param pParameterManger
   */
  AbstractParameter(const std::string & rName, ParameterManager * pParameterManger = NULL)  // NOLINT
  : m_Name(rName)
  {
    // if parameter manager is provided add myself to it!
    if (pParameterManger != NULL) {
      pParameterManger->Add(this);
    }
  }

  /**
   * Constructs a parameter with the given name and description
   * @param rName
   * @param rDescription
   * @param pParameterManger
   */
  AbstractParameter(
    const std::string & rName,
    const std::string & rDescription,
    ParameterManager * pParameterManger = NULL)
  : m_Name(rName),
    m_Description(rDescription)
  {
    // if parameter manager is provided add myself to it!
    if (pParameterManger != NULL) {
      pParameterManger->Add(this);
    }
  }

  /**
   * Destructor
   */
  virtual ~AbstractParameter()
  {
  }

public:
  /**
   * Gets the name of this object
   * @return name
   */
  inline const std::string & GetName() const
  {
    return m_Name;
  }

  /**
   * Returns the parameter description
   * @return parameter description
   */
  inline const std::string & GetDescription() const
  {
    return m_Description;
  }

  /**
   * Get parameter value as string.
   * @return value as string
   */
  virtual const std::string GetValueAsString() const = 0;

  /**
   * Set parameter value from string.
   * @param rStringValue value as string
   */
  virtual void SetValueFromString(const std::string & rStringValue) = 0;

  /**
   * Clones the parameter
   * @return clone
   */
  virtual AbstractParameter * Clone() = 0;

public:
  /**
   * Write this parameter onto output stream
   * @param rStream output stream
   * @param rParameter
   */
  friend std::ostream & operator<<(std::ostream & rStream, const AbstractParameter & rParameter)
  {
    rStream.precision(6);
    rStream.flags(std::ios::fixed);

    rStream << rParameter.GetName() << " = " << rParameter.GetValueAsString();
    return rStream;
  }

private:
  std::string m_Name;
  std::string m_Description;
  /**
 * Serialization: class AbstractParameter
 */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Name);
    ar & BOOST_SERIALIZATION_NVP(m_Description);
  }
};    // AbstractParameter
BOOST_SERIALIZATION_ASSUME_ABSTRACT(AbstractParameter)

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Parameter class
 */
template<typename T>
class Parameter : public AbstractParameter
{
public:
  /**
   * Parameter with given name and value
   * @param rName
   * @param value
   * @param pParameterManger
   */
  Parameter()
  {
  }
  Parameter(const std::string & rName, T value, ParameterManager * pParameterManger = NULL)
  : AbstractParameter(rName, pParameterManger),
    m_Value(value)
  {
  }

  /**
   * Parameter with given name, description and value
   * @param rName
   * @param rDescription
   * @param value
   * @param pParameterManger
   */
  Parameter(
    const std::string & rName,
    const std::string & rDescription,
    T value,
    ParameterManager * pParameterManger = NULL)
  : AbstractParameter(rName, rDescription, pParameterManger),
    m_Value(value)
  {
  }

  /**
   * Destructor
   */
  virtual ~Parameter()
  {
  }

public:
  /**
   * Gets value of parameter
   * @return parameter value
   */
  inline const T & GetValue() const
  {
    return m_Value;
  }

  /**
   * Sets value of parameter
   * @param rValue
   */
  inline void SetValue(const T & rValue)
  {
    m_Value = rValue;
  }

  /**
   * Gets value of parameter as string
   * @return string version of value
   */
  virtual const std::string GetValueAsString() const
  {
    std::stringstream converter;
    converter << m_Value;
    return converter.str();
  }

  /**
   * Sets value of parameter from string
   * @param rStringValue
   */
  virtual void SetValueFromString(const std::string & rStringValue)
  {
    std::stringstream converter;
    converter.str(rStringValue);
    converter >> m_Value;
  }

  /**
   * Clone this parameter
   * @return clone of this parameter
   */
  virtual Parameter * Clone()
  {
    return new Parameter(GetName(), GetDescription(), GetValue());
  }

public:
  /**
   * Assignment operator
   */
  Parameter & operator=(const Parameter & rOther)
  {
    m_Value = rOther.m_Value;

    return *this;
  }

  /**
   * Sets the value of this parameter to given value
   */
  T operator=(T value)
  {
    m_Value = value;

    return m_Value;
  }

  /**
   * Serialization: class Parameter
   */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(AbstractParameter);
    ar & BOOST_SERIALIZATION_NVP(m_Value);
  }

protected:
  /**
   * Parameter value
   */
  T m_Value;
};    // Parameter
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Parameter)

template<>
inline void Parameter<kt_double>::SetValueFromString(const std::string & rStringValue)
{
  int precision = std::numeric_limits<double>::digits10;
  std::stringstream converter;
  converter.precision(precision);

  converter.str(rStringValue);

  m_Value = 0.0;
  converter >> m_Value;
}

template<>
inline const std::string Parameter<kt_double>::GetValueAsString() const
{
  std::stringstream converter;
  converter.precision(std::numeric_limits<double>::digits10);
  converter << m_Value;
  return converter.str();
}

template<>
inline void Parameter<kt_bool>::SetValueFromString(const std::string & rStringValue)
{
  if (rStringValue == "true" || rStringValue == "TRUE") {
    m_Value = true;
  } else {
    m_Value = false;
  }
}

template<>
inline const std::string Parameter<kt_bool>::GetValueAsString() const
{
  if (m_Value == true) {
    return "true";
  }

  return "false";
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Parameter enum class
 */
class ParameterEnum : public Parameter<kt_int32s>
{
  typedef std::map<std::string, kt_int32s> EnumMap;

public:
  /**
   * Construct a Parameter object with name and value
   * @param rName parameter name
   * @param value of parameter
   * @param pParameterManger
   */
  ParameterEnum(
    const std::string & rName, kt_int32s value,
    ParameterManager * pParameterManger = NULL)
  : Parameter<kt_int32s>(rName, value, pParameterManger)
  {
  }
  ParameterEnum()
  {
  }

  /**
   * Destructor
   */
  virtual ~ParameterEnum()
  {
  }

public:
  /**
   * Return a clone of this instance
   * @return clone
   */
  virtual Parameter<kt_int32s> * Clone()
  {
    ParameterEnum * pEnum = new ParameterEnum(GetName(), GetValue());

    pEnum->m_EnumDefines = m_EnumDefines;

    return pEnum;
  }

  /**
   * Set parameter value from string.
   * @param rStringValue value as string
   */
  virtual void SetValueFromString(const std::string & rStringValue)
  {
    if (m_EnumDefines.find(rStringValue) != m_EnumDefines.end()) {
      m_Value = m_EnumDefines[rStringValue];
    } else {
      std::string validValues;

      const_forEach(EnumMap, &m_EnumDefines)
      {
        validValues += iter->first + ", ";
      }

      throw Exception("Unable to set enum: " + rStringValue + ". Valid values are: " + validValues);
    }
  }

  /**
   * Get parameter value as string.
   * @return value as string
   */
  virtual const std::string GetValueAsString() const
  {
    const_forEach(EnumMap, &m_EnumDefines)
    {
      if (iter->second == m_Value) {
        return iter->first;
      }
    }

    throw Exception("Unable to lookup enum");
  }

  /**
   * Defines the enum with the given name as having the given value
   * @param value
   * @param rName
   */
  void DefineEnumValue(kt_int32s value, const std::string & rName)
  {
    if (m_EnumDefines.find(rName) == m_EnumDefines.end()) {
      m_EnumDefines[rName] = value;
    } else {
      std::cerr << "Overriding enum value: " << m_EnumDefines[rName] << " with " << value <<
        std::endl;

      m_EnumDefines[rName] = value;

      assert(false);
    }
  }

public:
  /**
   * Assignment operator
   */
  ParameterEnum & operator=(const ParameterEnum & rOther)
  {
    SetValue(rOther.GetValue());

    return *this;
  }

  /**
   * Assignment operator
   */
  kt_int32s operator=(kt_int32s value)
  {
    SetValue(value);

    return m_Value;
  }

private:
  EnumMap m_EnumDefines;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Parameter<kt_int32s>);
    ar & BOOST_SERIALIZATION_NVP(m_EnumDefines);
  }
};    // ParameterEnum
BOOST_SERIALIZATION_ASSUME_ABSTRACT(ParameterEnum)
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Set of parameters
 */
class Parameters : public Object
{
public:
  // @cond EXCLUDE
  KARTO_Object(Parameters)
  // @endcond

public:
  /**
   * Parameters
   * @param rName
   */
  Parameters()
  {
  }

  Parameters(const std::string & rName)  // NOLINT
  : Object(rName)
  {
  }

  /**
   * Destructor
   */
  virtual ~Parameters()
  {
  }

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Object);
  }

private:
  Parameters(const Parameters &);
  const Parameters & operator=(const Parameters &);
};    // Parameters

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class SensorData;

/**
 * Abstract Sensor base class
 */
class KARTO_EXPORT Sensor : public Object
{
  /**
   * Serialization: class Sensor
   */

public:
  Sensor()
  {
  }
  // @cond EXCLUDE
  KARTO_Object(Sensor)
  // @endcond

protected:
  /**
   * Construct a Sensor
   * @param rName sensor name
   */
  Sensor(const Name & rName);  // NOLINT

public:
  /**
   * Destructor
   */
  virtual ~Sensor();

public:
  /**
   * Gets this range finder sensor's offset
   * @return offset pose
   */
  inline const Pose2 & GetOffsetPose() const
  {
    return m_pOffsetPose->GetValue();
  }

  /**
   * Sets this range finder sensor's offset
   * @param rPose
   */
  inline void SetOffsetPose(const Pose2 & rPose)
  {
    m_pOffsetPose->SetValue(rPose);
  }

  /**
   * Validates sensor
   * @return true if valid
   */
  virtual kt_bool Validate() = 0;

  /**
   * Validates sensor data
   * @param pSensorData sensor data
   * @return true if valid
   */
  virtual kt_bool Validate(SensorData * pSensorData) = 0;

private:
  /**
   * Restrict the copy constructor
   */
  Sensor(const Sensor &);  // NOLINT

  /**
   * Restrict the assignment operator
   */
  const Sensor & operator=(const Sensor &);

private:
  /**
   * Sensor offset pose
   */
  Parameter<Pose2> * m_pOffsetPose;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Object);
    ar & BOOST_SERIALIZATION_NVP(m_pOffsetPose);
  }
};    // Sensor
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Sensor)
/**
 * Type declaration of Sensor vector
 */
typedef std::vector<Sensor *> SensorVector;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Type declaration of <Name, Sensor*> map
 */
typedef std::map<Name, Sensor *> SensorManagerMap;

/**
 * Manages sensors
 */
class KARTO_EXPORT SensorManager
{
public:
  /**
   * Constructor
   */
  SensorManager()
  {
  }

  /**
   * Destructor
   */
  virtual ~SensorManager()
  {
  }

public:
  /**
   * Get singleton instance of SensorManager
   */
  static SensorManager * GetInstance();

public:
  /**
   * Registers a sensor by it's name. The Sensor name must be unique, if not sensor is not registered
   * unless override is set to true
   * @param pSensor sensor to register
   * @param override
   * @return true if sensor is registered with SensorManager, false if Sensor name is not unique
   */
  void RegisterSensor(Sensor * pSensor, kt_bool override = false)
  {
    Validate(pSensor);

    if ((m_Sensors.find(pSensor->GetName()) != m_Sensors.end()) && !override) {
      throw Exception("Cannot register sensor: already registered: [" +
              pSensor->GetName().ToString() +
              "] (Consider setting 'override' to true)");
    }

    std::cout << "Registering sensor: [" << pSensor->GetName().ToString() << "]" << std::endl;

    m_Sensors[pSensor->GetName()] = pSensor;
  }

  /**
   * Unregisters the given sensor
   * @param pSensor sensor to unregister
   */
  void UnregisterSensor(Sensor * pSensor)
  {
    Validate(pSensor);

    if (m_Sensors.find(pSensor->GetName()) != m_Sensors.end()) {
      std::cout << "Unregistering sensor: " << pSensor->GetName().ToString() << std::endl;

      m_Sensors.erase(pSensor->GetName());
    } else {
      throw Exception(
              "Cannot unregister sensor: not registered: [" + pSensor->GetName().ToString() + "]");
    }
  }

  /**
   * Gets the sensor with the given name
   * @param rName name of sensor
   * @return sensor
   */
  Sensor * GetSensorByName(const Name & rName)
  {
    if (m_Sensors.find(rName) != m_Sensors.end()) {
      return m_Sensors[rName];
    }

    throw Exception(
            "Sensor not registered: [" + rName.ToString() +
            "] (Did you add the sensor to the Dataset?)");
  }

  /**
   * Gets the sensor with the given name
   * @param rName name of sensor
   * @return sensor
   */
  template<class T>
  T * GetSensorByName(const Name & rName)
  {
    Sensor * pSensor = GetSensorByName(rName);

    return dynamic_cast<T *>(pSensor);
  }

  /**
   * Gets all registered sensors
   * @return vector of all registered sensors
   */
  SensorVector GetAllSensors()
  {
    SensorVector sensors;

    forEach(SensorManagerMap, &m_Sensors)
    {
      sensors.push_back(iter->second);
    }

    return sensors;
  }

protected:
  /**
   * Checks that given sensor is not NULL and has non-empty name
   * @param pSensor sensor to validate
   */
  static void Validate(Sensor * pSensor)
  {
    if (pSensor == NULL) {
      throw Exception("Invalid sensor:  NULL");
    } else if (pSensor->GetName().ToString() == "") {
      throw Exception("Invalid sensor:  nameless");
    }
  }

protected:
  /**
   * Sensor map
   */
  SensorManagerMap m_Sensors;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Sensors);
  }
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Sensor that provides pose information relative to world coordinates.
 *
 * The user can set the offset pose of the drive sensor.  If no value is provided by the user the default is no offset,
 * i.e, the sensor is initially at the world origin, oriented along the positive z axis.
 */
class Drive : public Sensor
{
public:
  // @cond EXCLUDE
  KARTO_Object(Drive)
  // @endcond

public:
  /**
   * Constructs a Drive object
   */
  Drive(const std::string & rName)  // NOLINT
  : Sensor(rName)
  {
  }
  /**
   * Destructor
   */
  virtual ~Drive()
  {
  }

public:
  virtual kt_bool Validate()
  {
    return true;
  }

  virtual kt_bool Validate(SensorData * pSensorData)
  {
    if (pSensorData == NULL) {
      return false;
    }

    return true;
  }

private:
  Drive(const Drive &);
  const Drive & operator=(const Drive &);
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Sensor);
  }
};    // class Drive

BOOST_SERIALIZATION_ASSUME_ABSTRACT(Drive)
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class LocalizedRangeScan;
class CoordinateConverter;

/**
 * The LaserRangeFinder defines a laser sensor that provides the pose offset position of a localized range scan relative to the robot.
 * The user can set an offset pose for the sensor relative to the robot coordinate system. If no value is provided
 * by the user, the sensor is set to be at the origin of the robot coordinate system.
 * The LaserRangeFinder contains parameters for physical laser sensor used by the mapper for scan matching
 * Also contains information about the maximum range of the sensor and provides a threshold
 * for limiting the range of readings.
 * The optimal value for the range threshold depends on the angular resolution of the scan and
 * the desired map resolution.  RangeThreshold should be set as large as possible while still
 * providing "solid" coverage between consecutive range readings.  The diagram below illustrates
 * the relationship between map resolution and the range threshold.
 */
class KARTO_EXPORT LaserRangeFinder : public Sensor
{
public:
  LaserRangeFinder()
  {
  }
  // @cond EXCLUDE
  KARTO_Object(LaserRangeFinder)
  // @endcond

public:
  /**
   * Destructor
   */
  virtual ~LaserRangeFinder()
  {
  }

public:
  /**
   * Gets this range finder sensor's minimum range
   * @return minimum range
   */
  inline kt_double GetMinimumRange() const
  {
    return m_pMinimumRange->GetValue();
  }

  /**
   * Sets this range finder sensor's minimum range
   * @param minimumRange
   */
  inline void SetMinimumRange(kt_double minimumRange)
  {
    m_pMinimumRange->SetValue(minimumRange);

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets this range finder sensor's maximum range
   * @return maximum range
   */
  inline kt_double GetMaximumRange() const
  {
    return m_pMaximumRange->GetValue();
  }

  /**
   * Sets this range finder sensor's maximum range
   * @param maximumRange
   */
  inline void SetMaximumRange(kt_double maximumRange)
  {
    m_pMaximumRange->SetValue(maximumRange);

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets the range threshold
   * @return range threshold
   */
  inline kt_double GetRangeThreshold() const
  {
    return m_pRangeThreshold->GetValue();
  }

  /**
   * Sets the range threshold
   * @param rangeThreshold
   */
  inline void SetRangeThreshold(kt_double rangeThreshold)
  {
    // make sure rangeThreshold is within laser range finder range
    m_pRangeThreshold->SetValue(math::Clip(rangeThreshold, GetMinimumRange(), GetMaximumRange()));

    if (math::DoubleEqual(GetRangeThreshold(), rangeThreshold) == false) {
      std::cout << "Info: clipped range threshold to be within minimum and maximum range!" <<
        std::endl;
    }
  }

  /**
   * Gets this range finder sensor's minimum angle
   * @return minimum angle
   */
  inline kt_double GetMinimumAngle() const
  {
    return m_pMinimumAngle->GetValue();
  }

  /**
   * Sets this range finder sensor's minimum angle
   * @param minimumAngle
   */
  inline void SetMinimumAngle(kt_double minimumAngle)
  {
    m_pMinimumAngle->SetValue(minimumAngle);

    Update();
  }

  /**
   * Gets this range finder sensor's maximum angle
   * @return maximum angle
   */
  inline kt_double GetMaximumAngle() const
  {
    return m_pMaximumAngle->GetValue();
  }

  /**
   * Sets this range finder sensor's maximum angle
   * @param maximumAngle
   */
  inline void SetMaximumAngle(kt_double maximumAngle)
  {
    m_pMaximumAngle->SetValue(maximumAngle);

    Update();
  }

  /**
   * Gets this range finder sensor's angular resolution
   * @return angular resolution
   */
  inline kt_double GetAngularResolution() const
  {
    return m_pAngularResolution->GetValue();
  }

  /**
   * Sets this range finder sensor's angular resolution
   * @param angularResolution
   */
  inline void SetAngularResolution(kt_double angularResolution)
  {
    if (m_pType->GetValue() == LaserRangeFinder_Custom) {
      m_pAngularResolution->SetValue(angularResolution);
    } else if (m_pType->GetValue() == LaserRangeFinder_Sick_LMS100) {
      if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.25))) {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.25));
      } else if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.50))) {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.50));
      } else {
        std::stringstream stream;
        stream << "Invalid resolution for Sick LMS100:  ";
        stream << angularResolution;
        throw Exception(stream.str());
      }
    } else if (m_pType->GetValue() == LaserRangeFinder_Sick_LMS200 ||  // NOLINT
      m_pType->GetValue() == LaserRangeFinder_Sick_LMS291)
    {
      if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.25))) {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.25));
      } else if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.50))) {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.50));
      } else if (math::DoubleEqual(angularResolution, math::DegreesToRadians(1.00))) {
        m_pAngularResolution->SetValue(math::DegreesToRadians(1.00));
      } else {
        std::stringstream stream;
        stream << "Invalid resolution for Sick LMS291:  ";
        stream << angularResolution;
        throw Exception(stream.str());
      }
    } else {
      throw Exception(
              "Can't set angular resolution, please create a LaserRangeFinder of type Custom");
    }

    Update();
  }

  /**
   * Return Laser type
   */
  inline kt_int32s GetType()
  {
    return m_pType->GetValue();
  }

  /**
   * Gets the number of range readings each localized range scan must contain to be a valid scan.
   * @return number of range readings
   */
  inline kt_int32u GetNumberOfRangeReadings() const
  {
    return m_NumberOfRangeReadings;
  }

  /**
   * Gets if this range finder sensor is 360° laser
   * @return is360Laser
   */
  inline kt_bool GetIs360Laser() const
  {
    return m_pIs360Laser->GetValue();
  }

  /**
   * Sets if this range finder sensor is 360° laser
   * @param is360Laser
   */
  inline void SetIs360Laser(bool is_360_laser)
  {
    m_pIs360Laser->SetValue(is_360_laser);

    Update();
  }

  virtual kt_bool Validate()
  {
    Update();

    if (math::InRange(GetRangeThreshold(), GetMinimumRange(), GetMaximumRange()) == false) {
      std::cout << "Please set range threshold to a value between [" <<
        GetMinimumRange() << ";" << GetMaximumRange() << "]" << std::endl;
      return false;
    }

    return true;
  }

  virtual kt_bool Validate(SensorData * pSensorData);

  /**
   * Get point readings (potentially scale readings if given coordinate converter is not null)
   * @param pLocalizedRangeScan
   * @param pCoordinateConverter
   * @param ignoreThresholdPoints
   * @param flipY
   */
  const PointVectorDouble GetPointReadings(
    LocalizedRangeScan * pLocalizedRangeScan,
    CoordinateConverter * pCoordinateConverter,
    kt_bool ignoreThresholdPoints = true,
    kt_bool flipY = false) const;

public:
  /**
   * Create a laser range finder of the given type and ID
   * @param type
   * @param rName name of sensor - if no name is specified default name will be assigned
   * @return laser range finder
   */
  static LaserRangeFinder * CreateLaserRangeFinder(LaserRangeFinderType type, const Name & rName)
  {
    LaserRangeFinder * pLrf = NULL;

    switch (type) {
      // see http://www.hizook.com/files/publications/SICK_LMS100.pdf
      // set range threshold to 18m
      case LaserRangeFinder_Sick_LMS100:
        {
          pLrf = new LaserRangeFinder((rName.GetName() != "") ? rName : Name("Sick LMS 100"));

          // Sensing range is 18 meters (at 10% reflectivity,
          // max range of 20 meters), with an error of about 20mm
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(20.0);

          // 270 degree range, 50 Hz
          pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-135));
          pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(135));

          // 0.25 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.25));

          pLrf->m_NumberOfRangeReadings = 1081;
        }
        break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 10m
      case LaserRangeFinder_Sick_LMS200:
        {
          pLrf = new LaserRangeFinder((rName.GetName() != "") ? rName : Name("Sick LMS 200"));

          // Sensing range is 80 meters
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(80.0);

          // 180 degree range, 75 Hz
          pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-90));
          pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(90));

          // 0.5 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.5));

          pLrf->m_NumberOfRangeReadings = 361;
        }
        break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 30m
      case LaserRangeFinder_Sick_LMS291:
        {
          pLrf = new LaserRangeFinder((rName.GetName() != "") ? rName : Name("Sick LMS 291"));

          // Sensing range is 80 meters
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(80.0);

          // 180 degree range, 75 Hz
          pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-90));
          pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(90));

          // 0.5 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.5));

          pLrf->m_NumberOfRangeReadings = 361;
        }
        break;

      // see http://www.hizook.com/files/publications/Hokuyo_UTM_LaserRangeFinder_LIDAR.pdf
      // set range threshold to 30m
      case LaserRangeFinder_Hokuyo_UTM_30LX:
        {
          pLrf = new LaserRangeFinder((rName.GetName() != "") ? rName : Name("Hokuyo UTM-30LX"));

          // Sensing range is 30 meters
          pLrf->m_pMinimumRange->SetValue(0.1);
          pLrf->m_pMaximumRange->SetValue(30.0);

          // 270 degree range, 40 Hz
          pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-135));
          pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(135));

          // 0.25 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.25));

          pLrf->m_NumberOfRangeReadings = 1081;
        }
        break;

      // see http://www.hizook.com/files/publications/HokuyoURG_Datasheet.pdf
      // set range threshold to 3.5m
      case LaserRangeFinder_Hokuyo_URG_04LX:
        {
          pLrf = new LaserRangeFinder((rName.GetName() != "") ? rName : Name("Hokuyo URG-04LX"));

          // Sensing range is 4 meters. It has detection problems when
          // scanning absorptive surfaces (such as black trimming).
          pLrf->m_pMinimumRange->SetValue(0.02);
          pLrf->m_pMaximumRange->SetValue(4.0);

          // 240 degree range, 10 Hz
          pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-120));
          pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(120));

          // 0.352 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.352));

          pLrf->m_NumberOfRangeReadings = 751;
        }
        break;

      case LaserRangeFinder_Custom:
        {
          pLrf =
            new LaserRangeFinder((rName.GetName() != "") ? rName : Name(
                "User-Defined LaserRangeFinder"));

          // Sensing range is 80 meters.
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(80.0);

          // 180 degree range
          pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-90));
          pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(90));

          // 1.0 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(1.0));

          pLrf->m_NumberOfRangeReadings = 181;
        }
        break;
    }

    if (pLrf != NULL) {
      pLrf->m_pType->SetValue(type);

      Pose2 defaultOffset;
      pLrf->SetOffsetPose(defaultOffset);
    }

    return pLrf;
  }

private:
  /**
   * Constructs a LaserRangeFinder object with given ID
   */
  LaserRangeFinder(const Name & rName)  // NOLINT
  : Sensor(rName),
    m_NumberOfRangeReadings(0)
  {
    m_pMinimumRange = new Parameter<kt_double>("MinimumRange", 0.0, GetParameterManager());
    m_pMaximumRange = new Parameter<kt_double>("MaximumRange", 80.0, GetParameterManager());

    m_pMinimumAngle = new Parameter<kt_double>("MinimumAngle", -KT_PI_2, GetParameterManager());
    m_pMaximumAngle = new Parameter<kt_double>("MaximumAngle", KT_PI_2, GetParameterManager());

    m_pAngularResolution = new Parameter<kt_double>("AngularResolution",
        math::DegreesToRadians(1),
        GetParameterManager());

    m_pRangeThreshold = new Parameter<kt_double>("RangeThreshold", 12.0, GetParameterManager());

    m_pIs360Laser = new Parameter<kt_bool>("Is360DegreeLaser", false, GetParameterManager());

    m_pType = new ParameterEnum("Type", LaserRangeFinder_Custom, GetParameterManager());
    m_pType->DefineEnumValue(LaserRangeFinder_Custom, "Custom");
    m_pType->DefineEnumValue(LaserRangeFinder_Sick_LMS100, "Sick_LMS100");
    m_pType->DefineEnumValue(LaserRangeFinder_Sick_LMS200, "Sick_LMS200");
    m_pType->DefineEnumValue(LaserRangeFinder_Sick_LMS291, "Sick_LMS291");
    m_pType->DefineEnumValue(LaserRangeFinder_Hokuyo_UTM_30LX, "Hokuyo_UTM_30LX");
    m_pType->DefineEnumValue(LaserRangeFinder_Hokuyo_URG_04LX, "Hokuyo_URG_04LX");
  }

  /**
   * Set the number of range readings based on the minimum and
   * maximum angles of the sensor and the angular resolution
   */
  void Update()
  {
    int residual = 1;
    if (GetIs360Laser()) {
      // residual is 0 by 360 lidar conventions
      residual = 0;
    }

    m_NumberOfRangeReadings = static_cast<kt_int32u>(math::Round((GetMaximumAngle() -
      GetMinimumAngle()) /
      GetAngularResolution()) + residual);
  }

private:
  LaserRangeFinder(const LaserRangeFinder &);
  const LaserRangeFinder & operator=(const LaserRangeFinder &);

private:
  // sensor m_Parameters
  Parameter<kt_double> * m_pMinimumAngle;
  Parameter<kt_double> * m_pMaximumAngle;

  Parameter<kt_double> * m_pAngularResolution;

  Parameter<kt_double> * m_pMinimumRange;
  Parameter<kt_double> * m_pMaximumRange;

  Parameter<kt_double> * m_pRangeThreshold;

  Parameter<kt_bool> * m_pIs360Laser;

  ParameterEnum * m_pType;

  kt_int32u m_NumberOfRangeReadings;

  // static std::string LaserRangeFinderTypeNames[6];
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    if (Archive::is_loading::value) {
      m_pMinimumRange = new Parameter<kt_double>("MinimumRange", 0.0, GetParameterManager());
      m_pMaximumRange = new Parameter<kt_double>("MaximumRange", 80.0, GetParameterManager());

      m_pMinimumAngle = new Parameter<kt_double>("MinimumAngle", -KT_PI_2, GetParameterManager());
      m_pMaximumAngle = new Parameter<kt_double>("MaximumAngle", KT_PI_2, GetParameterManager());

      m_pAngularResolution = new Parameter<kt_double>("AngularResolution",
          math::DegreesToRadians(1),
          GetParameterManager());

      m_pRangeThreshold = new Parameter<kt_double>("RangeThreshold", 12.0, GetParameterManager());

      m_pIs360Laser = new Parameter<kt_bool>("Is360Laser", false, GetParameterManager());

      m_pType = new ParameterEnum("Type", LaserRangeFinder_Custom, GetParameterManager());
    }

    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Sensor);
    ar & BOOST_SERIALIZATION_NVP(m_pMinimumAngle);
    ar & BOOST_SERIALIZATION_NVP(m_pMaximumAngle);
    ar & BOOST_SERIALIZATION_NVP(m_pAngularResolution);
    ar & BOOST_SERIALIZATION_NVP(m_pMinimumRange);
    ar & BOOST_SERIALIZATION_NVP(m_pMaximumRange);
    ar & BOOST_SERIALIZATION_NVP(m_pRangeThreshold);
    ar & BOOST_SERIALIZATION_NVP(m_pIs360Laser);
    ar & BOOST_SERIALIZATION_NVP(m_pType);
    ar & BOOST_SERIALIZATION_NVP(m_NumberOfRangeReadings);
  }
};    // LaserRangeFinder
BOOST_SERIALIZATION_ASSUME_ABSTRACT(LaserRangeFinder)
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Enumerated type for valid grid cell states
 */
typedef enum
{
  GridStates_Unknown = 0,
  GridStates_Occupied = 100,
  GridStates_Free = 255
} GridStates;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The CoordinateConverter class is used to convert coordinates between world and grid coordinates
 * In world coordinates 1.0 = 1 meter where 1 in grid coordinates = 1 pixel!
 * Default scale for coordinate converter is 20 that converters to 1 pixel = 0.05 meter
 */
class CoordinateConverter
{
public:
  /**
   * Default constructor
   */
  CoordinateConverter()
  : m_Scale(20.0)
  {
  }

public:
  /**
   * Scales the value
   * @param value
   * @return scaled value
   */
  inline kt_double Transform(kt_double value)
  {
    return value * m_Scale;
  }

  /**
   * Converts the point from world coordinates to grid coordinates
   * @param rWorld world coordinate
   * @param flipY
   * @return grid coordinate
   */
  inline Vector2<kt_int32s> WorldToGrid(
    const Vector2<kt_double> & rWorld,
    kt_bool flipY = false) const
  {
    kt_double gridX = (rWorld.GetX() - m_Offset.GetX()) * m_Scale;
    kt_double gridY = 0.0;

    if (flipY == false) {
      gridY = (rWorld.GetY() - m_Offset.GetY()) * m_Scale;
    } else {
      gridY = (m_Size.GetHeight() / m_Scale - rWorld.GetY() + m_Offset.GetY()) * m_Scale;
    }

    return Vector2<kt_int32s>(static_cast<kt_int32s>(math::Round(gridX)),
             static_cast<kt_int32s>(math::Round(gridY)));
  }

  /**
   * Converts the point from grid coordinates to world coordinates
   * @param rGrid world coordinate
   * @param flipY
   * @return world coordinate
   */
  inline Vector2<kt_double> GridToWorld(
    const Vector2<kt_int32s> & rGrid,
    kt_bool flipY = false) const
  {
    kt_double worldX = m_Offset.GetX() + rGrid.GetX() / m_Scale;
    kt_double worldY = 0.0;

    if (flipY == false) {
      worldY = m_Offset.GetY() + rGrid.GetY() / m_Scale;
    } else {
      worldY = m_Offset.GetY() + (m_Size.GetHeight() - rGrid.GetY()) / m_Scale;
    }

    return Vector2<kt_double>(worldX, worldY);
  }

  /**
   * Gets the scale
   * @return scale
   */
  inline kt_double GetScale() const
  {
    return m_Scale;
  }

  /**
   * Sets the scale
   * @param scale
   */
  inline void SetScale(kt_double scale)
  {
    m_Scale = scale;
  }

  /**
   * Gets the offset
   * @return offset
   */
  inline const Vector2<kt_double> & GetOffset() const
  {
    return m_Offset;
  }

  /**
   * Sets the offset
   * @param rOffset
   */
  inline void SetOffset(const Vector2<kt_double> & rOffset)
  {
    m_Offset = rOffset;
  }

  /**
   * Sets the size
   * @param rSize
   */
  inline void SetSize(const Size2<kt_int32s> & rSize)
  {
    m_Size = rSize;
  }

  /**
   * Gets the size
   * @return size
   */
  inline const Size2<kt_int32s> & GetSize() const
  {
    return m_Size;
  }

  /**
   * Gets the resolution
   * @return resolution
   */
  inline kt_double GetResolution() const
  {
    return 1.0 / m_Scale;
  }

  /**
   * Sets the resolution
   * @param resolution
   */
  inline void SetResolution(kt_double resolution)
  {
    m_Scale = 1.0 / resolution;
  }

  /**
   * Gets the bounding box
   * @return bounding box
   */
  inline BoundingBox2 GetBoundingBox() const
  {
    BoundingBox2 box;

    kt_double minX = GetOffset().GetX();
    kt_double minY = GetOffset().GetY();
    kt_double maxX = minX + GetSize().GetWidth() * GetResolution();
    kt_double maxY = minY + GetSize().GetHeight() * GetResolution();

    box.SetMinimum(GetOffset());
    box.SetMaximum(Vector2<kt_double>(maxX, maxY));
    return box;
  }

private:
  Size2<kt_int32s> m_Size;
  kt_double m_Scale;

  Vector2<kt_double> m_Offset;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Size);
    ar & BOOST_SERIALIZATION_NVP(m_Scale);
    ar & BOOST_SERIALIZATION_NVP(m_Offset);
  }
};    // CoordinateConverter

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a grid class
 */
template<typename T>
class Grid
{
public:
  /**
   * Creates a grid of given size and resolution
   * @param width
   * @param height
   * @param resolution
   * @return grid pointer
   */
  Grid()
  {
  }
  static Grid * CreateGrid(kt_int32s width, kt_int32s height, kt_double resolution)
  {
    Grid * pGrid = new Grid(width, height);

    pGrid->GetCoordinateConverter()->SetScale(1.0 / resolution);

    return pGrid;
  }

  /**
   * Destructor
   */
  virtual ~Grid()
  {
    if (m_pData) {
      delete[] m_pData;
    }
    if (m_pCoordinateConverter) {
      delete m_pCoordinateConverter;
    }
  }

public:
  /**
   * Clear out the grid data
   */
  void Clear()
  {
    memset(m_pData, 0, GetDataSize() * sizeof(T));
  }

  /**
   * Returns a clone of this grid
   * @return grid clone
   */
  Grid * Clone()
  {
    Grid * pGrid = CreateGrid(GetWidth(), GetHeight(), GetResolution());
    pGrid->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    memcpy(pGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    return pGrid;
  }

  /**
   * Resizes the grid (deletes all old data)
   * @param width
   * @param height
   */
  virtual void Resize(kt_int32s width, kt_int32s height)
  {
    m_Width = width;
    m_Height = height;
    m_WidthStep = math::AlignValue<kt_int32s>(width, 8);

    if (m_pData != NULL) {
      delete[] m_pData;
      m_pData = NULL;
    }

    try {
      m_pData = new T[GetDataSize()];

      if (m_pCoordinateConverter == NULL) {
        m_pCoordinateConverter = new CoordinateConverter();
      }

      m_pCoordinateConverter->SetSize(Size2<kt_int32s>(width, height));
    } catch (...) {
      m_pData = NULL;

      m_Width = 0;
      m_Height = 0;
      m_WidthStep = 0;
    }

    Clear();
  }

  /**
   * Checks whether the given coordinates are valid grid indices
   * @param rGrid
   */
  inline kt_bool IsValidGridIndex(const Vector2<kt_int32s> & rGrid) const
  {
    return math::IsUpTo(rGrid.GetX(), m_Width) && math::IsUpTo(rGrid.GetY(), m_Height);
  }

  /**
   * Gets the index into the data pointer of the given grid coordinate
   * @param rGrid
   * @param boundaryCheck default value is true
   * @return grid index
   */
  virtual kt_int32s GridIndex(const Vector2<kt_int32s> & rGrid, kt_bool boundaryCheck = true) const
  {
    if (boundaryCheck == true) {
      if (IsValidGridIndex(rGrid) == false) {
        std::stringstream error;
        error << "Index " << rGrid << " out of range.  Index must be between [0; " <<
          m_Width << ") and [0; " << m_Height << ")";
        throw Exception(error.str());
      }
    }

    kt_int32s index = rGrid.GetX() + (rGrid.GetY() * m_WidthStep);

    if (boundaryCheck == true) {
      assert(math::IsUpTo(index, GetDataSize()));
    }

    return index;
  }

  /**
   * Gets the grid coordinate from an index
   * @param index
   * @return grid coordinate
   */
  Vector2<kt_int32s> IndexToGrid(kt_int32s index) const
  {
    Vector2<kt_int32s> grid;

    grid.SetY(index / m_WidthStep);
    grid.SetX(index - grid.GetY() * m_WidthStep);

    return grid;
  }

  /**
   * Converts the point from world coordinates to grid coordinates
   * @param rWorld world coordinate
   * @param flipY
   * @return grid coordinate
   */
  inline Vector2<kt_int32s> WorldToGrid(
    const Vector2<kt_double> & rWorld,
    kt_bool flipY = false) const
  {
    return GetCoordinateConverter()->WorldToGrid(rWorld, flipY);
  }

  /**
   * Converts the point from grid coordinates to world coordinates
   * @param rGrid world coordinate
   * @param flipY
   * @return world coordinate
   */
  inline Vector2<kt_double> GridToWorld(
    const Vector2<kt_int32s> & rGrid,
    kt_bool flipY = false) const
  {
    return GetCoordinateConverter()->GridToWorld(rGrid, flipY);
  }

  /**
   * Gets pointer to data at given grid coordinate
   * @param rGrid grid coordinate
   * @return grid point
   */
  T * GetDataPointer(const Vector2<kt_int32s> & rGrid)
  {
    kt_int32s index = GridIndex(rGrid, true);
    return m_pData + index;
  }

  /**
   * Gets pointer to data at given grid coordinate
   * @param rGrid grid coordinate
   * @return grid point
   */
  T * GetDataPointer(const Vector2<kt_int32s> & rGrid) const
  {
    kt_int32s index = GridIndex(rGrid, true);
    return m_pData + index;
  }

  /**
   * Gets the width of the grid
   * @return width of the grid
   */
  inline kt_int32s GetWidth() const
  {
    return m_Width;
  }

  /**
   * Gets the height of the grid
   * @return height of the grid
   */
  inline kt_int32s GetHeight() const
  {
    return m_Height;
  }

  /**
   * Get the size as a Size2<kt_int32s>
   * @return size of the grid
   */
  inline const Size2<kt_int32s> GetSize() const
  {
    return Size2<kt_int32s>(m_Width, m_Height);
  }

  /**
   * Gets the width step in bytes
   * @return width step
   */
  inline kt_int32s GetWidthStep() const
  {
    return m_WidthStep;
  }

  /**
   * Gets the grid data pointer
   * @return data pointer
   */
  inline T * GetDataPointer()
  {
    return m_pData;
  }

  /**
   * Gets const grid data pointer
   * @return data pointer
   */
  inline T * GetDataPointer() const
  {
    return m_pData;
  }

  /**
   * Gets the allocated grid size in bytes
   * @return data size
   */
  inline kt_int32s GetDataSize() const
  {
    return m_WidthStep * m_Height;
  }

  /**
   * Get value at given grid coordinate
   * @param rGrid grid coordinate
   * @return value
   */
  inline T GetValue(const Vector2<kt_int32s> & rGrid) const
  {
    kt_int32s index = GridIndex(rGrid);
    return m_pData[index];
  }

  /**
   * Gets the coordinate converter for this grid
   * @return coordinate converter
   */
  inline CoordinateConverter * GetCoordinateConverter() const
  {
    return m_pCoordinateConverter;
  }

  /**
   * Gets the resolution
   * @return resolution
   */
  inline kt_double GetResolution() const
  {
    return GetCoordinateConverter()->GetResolution();
  }

  /**
   * Gets the grids bounding box
   * @return bounding box
   */
  inline BoundingBox2 GetBoundingBox() const
  {
    return GetCoordinateConverter()->GetBoundingBox();
  }

  /**
   * Increments all the grid cells from (x0, y0) to (x1, y1);
   * if applicable, apply f to each cell traced
   * @param x0
   * @param y0
   * @param x1
   * @param y1
   * @param f
   */
  void TraceLine(kt_int32s x0, kt_int32s y0, kt_int32s x1, kt_int32s y1, Functor * f = NULL)
  {
    kt_bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
      std::swap(x0, y0);
      std::swap(x1, y1);
    }
    if (x0 > x1) {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }

    kt_int32s deltaX = x1 - x0;
    kt_int32s deltaY = abs(y1 - y0);
    kt_int32s error = 0;
    kt_int32s ystep;
    kt_int32s y = y0;

    if (y0 < y1) {
      ystep = 1;
    } else {
      ystep = -1;
    }

    kt_int32s pointX;
    kt_int32s pointY;
    for (kt_int32s x = x0; x <= x1; x++) {
      if (steep) {
        pointX = y;
        pointY = x;
      } else {
        pointX = x;
        pointY = y;
      }

      error += deltaY;

      if (2 * error >= deltaX) {
        y += ystep;
        error -= deltaX;
      }

      Vector2<kt_int32s> gridIndex(pointX, pointY);
      if (IsValidGridIndex(gridIndex)) {
        kt_int32s index = GridIndex(gridIndex, false);
        T * pGridPointer = GetDataPointer();
        pGridPointer[index]++;

        if (f != NULL) {
          (*f)(index);
        }
      }
    }
  }

protected:
  /**
   * Constructs grid of given size
   * @param width
   * @param height
   */
  Grid(kt_int32s width, kt_int32s height)
  : m_pData(NULL),
    m_pCoordinateConverter(NULL)
  {
    Resize(width, height);
  }

private:
  kt_int32s m_Width;         // width of grid
  kt_int32s m_Height;        // height of grid
  kt_int32s m_WidthStep;     // 8 bit aligned width of grid
  T * m_pData;               // grid data

  // coordinate converter to convert between world coordinates and grid coordinates
  CoordinateConverter * m_pCoordinateConverter;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Width);
    ar & BOOST_SERIALIZATION_NVP(m_Height);
    ar & BOOST_SERIALIZATION_NVP(m_WidthStep);
    ar & BOOST_SERIALIZATION_NVP(m_pCoordinateConverter);


    if (Archive::is_loading::value) {
      m_pData = new T[m_WidthStep * m_Height];
    }
    ar & boost::serialization::make_array<T>(m_pData, m_WidthStep * m_Height);
  }
};    // Grid
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Grid)

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * For making custom data
 */
class CustomData : public Object
{
public:
  // @cond EXCLUDE
  KARTO_Object(CustomData)
  // @endcond

public:
  /**
   * Constructor
   */
  CustomData()
  : Object()
  {
  }

  /**
   * Destructor
   */
  virtual ~CustomData()
  {
  }

public:
  /**
   * Write out this custom data as a string
   * @return string representation of this data object
   */
  virtual const std::string Write() const = 0;

  /**
   * Read in this custom data from a string
   * @param rValue string representation of this data object
   */
  virtual void Read(const std::string & rValue) = 0;

private:
  CustomData(const CustomData &);
  const CustomData & operator=(const CustomData &);

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Object);
  }
};
BOOST_SERIALIZATION_ASSUME_ABSTRACT(CustomData)

/**
 * Type declaration of CustomData vector
 */
typedef std::vector<CustomData *> CustomDataVector;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * SensorData is a base class for all sensor data
 */
class KARTO_EXPORT SensorData : public Object
{
public:
  // @cond EXCLUDE
  KARTO_Object(SensorData)
  // @endcond

public:
  SensorData() {}
  /**
   * Destructor
   */
  virtual ~SensorData();

public:
  /**
   * Gets sensor data id
   * @return sensor id
   */
  inline kt_int32s GetStateId() const
  {
    return m_StateId;
  }

  /**
   * Sets sensor data id
   * @param stateId id
   */
  inline void SetStateId(kt_int32s stateId)
  {
    m_StateId = stateId;
  }

  /**
   * Gets sensor unique id
   * @return unique id
   */
  inline kt_int32s GetUniqueId() const
  {
    return m_UniqueId;
  }

  /**
   * Sets sensor unique id
   * @param uniqueId
   */
  inline void SetUniqueId(kt_int32u uniqueId)
  {
    m_UniqueId = uniqueId;
  }

  /**
   * Gets sensor data time
   * @return time
   */
  inline kt_double GetTime() const
  {
    return m_Time;
  }

  /**
   * Sets sensor data time
   * @param time
   */
  inline void SetTime(kt_double time)
  {
    m_Time = time;
  }

  /**
   * Get the sensor that created this sensor data
   * @return sensor
   */
  inline const Name & GetSensorName() const
  {
    return m_SensorName;
  }

  /**
   * Add a CustomData object to sensor data
   * @param pCustomData
   */
  inline void AddCustomData(CustomData * pCustomData)
  {
    m_CustomData.push_back(pCustomData);
  }

  /**
   * Get all custom data objects assigned to sensor data
   * @return CustomDataVector&
   */
  inline const CustomDataVector & GetCustomData() const
  {
    return m_CustomData;
  }

protected:
  /**
   * Construct a SensorData object with a sensor name
   */
  SensorData(const Name & rSensorName);  // NOLINT

private:
  /**
   * Restrict the copy constructor
   */
  SensorData(const SensorData &);  // NOLINT

  /**
   * Restrict the assignment operator
   */
  const SensorData & operator=(const SensorData &);

private:
  /**
   * ID unique to individual sensor
   */
  kt_int32s m_StateId;

  /**
   * ID unique across all sensor data
   */
  kt_int32s m_UniqueId;

  /**
   * Sensor that created this sensor data
   */
  Name m_SensorName;

  /**
   * Time the sensor data was created
   */
  kt_double m_Time;

  CustomDataVector m_CustomData;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_StateId);
    ar & BOOST_SERIALIZATION_NVP(m_UniqueId);
    ar & BOOST_SERIALIZATION_NVP(m_SensorName);
    ar & BOOST_SERIALIZATION_NVP(m_Time);
    ar & BOOST_SERIALIZATION_NVP(m_CustomData);
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Object);
  }
};


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Type declaration of range readings vector
 */
typedef std::vector<kt_double> RangeReadingsVector;

/**
 * LaserRangeScan representing the range readings from a laser range finder sensor.
 */
class LaserRangeScan : public SensorData
{
public:
  // @cond EXCLUDE
  KARTO_Object(LaserRangeScan)
  // @endcond

public:
  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   */
  LaserRangeScan(const Name & rSensorName)  // NOLINT
  : SensorData(rSensorName),
    m_pRangeReadings(NULL),
    m_NumberOfRangeReadings(0)
  {
  }

  LaserRangeScan()
  {
  }

  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   * @param rRangeReadings
   */
  LaserRangeScan(const Name & rSensorName, const RangeReadingsVector & rRangeReadings)
  : SensorData(rSensorName),
    m_pRangeReadings(NULL),
    m_NumberOfRangeReadings(0)
  {
    assert(rSensorName.ToString() != "");

    SetRangeReadings(rRangeReadings);
  }

  /**
   * Destructor
   */
  virtual ~LaserRangeScan()
  {
    delete[] m_pRangeReadings;
    m_pRangeReadings = nullptr;
  }

public:
  /**
   * Gets the range readings of this scan
   * @return range readings of this scan
   */
  inline kt_double * GetRangeReadings() const
  {
    return m_pRangeReadings;
  }

  inline RangeReadingsVector GetRangeReadingsVector() const
  {
    return RangeReadingsVector(m_pRangeReadings, m_pRangeReadings + m_NumberOfRangeReadings);
  }

  /**
   * Sets the range readings for this scan
   * @param rRangeReadings
   */
  inline void SetRangeReadings(const RangeReadingsVector & rRangeReadings)
  {
    // ignore for now! XXXMAE BUGUBUG 05/21/2010 << TODO(lucbettaieb): What the heck is this??
    // if (rRangeReadings.size() != GetNumberOfRangeReadings())
    // {
    //   std::stringstream error;
    //   error << "Given number of readings (" << rRangeReadings.size()
    //         << ") does not match expected number of range finder ("
    //         << GetNumberOfRangeReadings() << ")";
    //   throw Exception(error.str());
    // }

    if (!rRangeReadings.empty()) {
      if (rRangeReadings.size() != m_NumberOfRangeReadings) {
        // delete old readings
        delete[] m_pRangeReadings;

        // store size of array!
        m_NumberOfRangeReadings = static_cast<kt_int32u>(rRangeReadings.size());

        // allocate range readings
        m_pRangeReadings = new kt_double[m_NumberOfRangeReadings];
      }

      // copy readings
      kt_int32u index = 0;
      const_forEach(RangeReadingsVector, &rRangeReadings)
      {
        m_pRangeReadings[index++] = *iter;
      }
    } else {
      delete[] m_pRangeReadings;
      m_pRangeReadings = NULL;
    }
  }

  /**
   * Gets the laser range finder sensor that generated this scan
   * @return laser range finder sensor of this scan
   */
  inline LaserRangeFinder * GetLaserRangeFinder() const
  {
    return SensorManager::GetInstance()->GetSensorByName<LaserRangeFinder>(GetSensorName());
  }

  /**
   * Gets the number of range readings
   * @return number of range readings
   */
  inline kt_int32u GetNumberOfRangeReadings() const
  {
    return m_NumberOfRangeReadings;
  }

private:
  LaserRangeScan(const LaserRangeScan &);
  const LaserRangeScan & operator=(const LaserRangeScan &);

private:
  kt_double * m_pRangeReadings;
  kt_int32u m_NumberOfRangeReadings;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_NumberOfRangeReadings);
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SensorData);

    if (Archive::is_loading::value) {
      m_pRangeReadings = new kt_double[m_NumberOfRangeReadings];
    }
    ar & boost::serialization::make_array<kt_double>(m_pRangeReadings, m_NumberOfRangeReadings);
  }
};    // LaserRangeScan

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * DrivePose representing the pose value of a drive sensor.
 */
class DrivePose : public SensorData
{
public:
  // @cond EXCLUDE
  KARTO_Object(DrivePose)
  // @endcond

public:
  /**
   * Constructs a pose of the given drive sensor
   * @param rSensorName
   */
  DrivePose(const Name & rSensorName)  // NOLINT
  : SensorData(rSensorName)
  {
  }

  /**
   * Destructor
   */
  virtual ~DrivePose()
  {
  }

public:
  /**
   * Gets the odometric pose of this scan
   * @return odometric pose of this scan
   */
  inline const Pose3 & GetOdometricPose() const
  {
    return m_OdometricPose;
  }

  /**
   * Sets the odometric pose of this scan
   * @param rPose
   */
  inline void SetOdometricPose(const Pose3 & rPose)
  {
    m_OdometricPose = rPose;
  }

private:
  DrivePose(const DrivePose &);
  const DrivePose & operator=(const DrivePose &);

private:
  /**
   * Odometric pose of robot
   */
  Pose3 m_OdometricPose;
};    // class DrivePose

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The LocalizedRangeScan contains range data from a single sweep of a laser range finder sensor
 * in a two-dimensional space and position information. The odometer position is the position
 * reported by the robot when the range data was recorded. The corrected position is the position
 * calculated by the mapper (or localizer)
 */
class LocalizedRangeScan : public LaserRangeScan
{
public:
  // @cond EXCLUDE
  KARTO_Object(LocalizedRangeScan)
  // @endcond

public:
  /**
   * Constructs a range scan from the given range finder with the given readings
   */
  LocalizedRangeScan(const Name & rSensorName, const RangeReadingsVector & rReadings)
  : LaserRangeScan(rSensorName, rReadings),
    m_IsDirty(true)
  {
  }

  LocalizedRangeScan()
  {}

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScan()
  {
  }

private:
  mutable std::shared_mutex m_Lock;

public:
  /**
   * Gets the odometric pose of this scan
   * @return odometric pose of this scan
   */
  inline const Pose2 & GetOdometricPose() const
  {
    return m_OdometricPose;
  }

  /**
   * Sets the odometric pose of this scan
   * @param rPose
   */
  inline void SetOdometricPose(const Pose2 & rPose)
  {
    m_OdometricPose = rPose;
  }

  /**
   * Gets the (possibly corrected) robot pose at which this scan was taken.  The corrected robot pose of the scan
   * is usually set by an external module such as a localization or mapping module when it is determined
   * that the original pose was incorrect.  The external module will set the correct pose based on
   * additional sensor data and any context information it has.  If the pose has not been corrected,
   * a call to this method returns the same pose as GetOdometricPose().
   * @return corrected pose
   */
  inline const Pose2 & GetCorrectedPose() const
  {
    return m_CorrectedPose;
  }

  /**
   * Moves the scan by moving the robot pose to the given location.
   * @param rPose new pose of the robot of this scan
   */
  inline void SetCorrectedPose(const Pose2 & rPose)
  {
    m_CorrectedPose = rPose;

    m_IsDirty = true;
  }

  /**
   * Moves the scan by moving the robot pose to the given location and update point readings.
   * @param rPose new pose of the robot of this scan
   */
  inline void SetCorrectedPoseAndUpdate(const Pose2& rPose)
  {
    SetCorrectedPose(rPose);

    Update();
  }

  /**
   * Gets barycenter of point readings
   */
  inline const Pose2 & GetBarycenterPose() const
  {
    std::shared_lock<std::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      std::unique_lock<std::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return m_BarycenterPose;
  }

  inline void SetBarycenterPose(Pose2 & bcenter)
  {
    m_BarycenterPose = bcenter;
  }

  /**
   * Gets barycenter if the given parameter is true, otherwise returns the scanner pose
   * @param useBarycenter
   * @return barycenter if parameter is true, otherwise scanner pose
   */
  inline Pose2 GetReferencePose(kt_bool useBarycenter) const
  {
    std::shared_lock<std::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      std::unique_lock<std::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return useBarycenter ? GetBarycenterPose() : GetSensorPose();
  }

  /**
   * Computes the position of the sensor
   * @return scan pose
   */
  inline Pose2 GetSensorPose() const
  {
    return GetSensorAt(m_CorrectedPose);
  }

  inline void SetIsDirty(kt_bool & rIsDirty)
  {
    m_IsDirty = rIsDirty;
  }

  /**
   * Computes the robot pose given the corrected scan pose
   * @param rScanPose pose of the sensor
   */
  void SetSensorPose(const Pose2& rScanPose)
  {
    m_CorrectedPose = GetCorrectedAt(rScanPose);

    Update();
  }

  /**
   * Computes the position of the sensor if the robot were at the given pose
   * @param rPose
   * @return sensor pose
   */
  inline Pose2 GetSensorAt(const Pose2 & rPose) const
  {
    return Transform(rPose).TransformPose(GetLaserRangeFinder()->GetOffsetPose());
  }

  /**
   * @brief Computes the pose of the robot if the sensor were at the given pose
   * @param sPose sensor pose
   * @return robot pose
   */
  inline Pose2 GetCorrectedAt(const Pose2& sPose) const
  {
    Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
    kt_double offsetLength = deviceOffsetPose2.GetPosition().Length();
    kt_double offsetHeading = deviceOffsetPose2.GetHeading();
    kt_double angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
    kt_double correctedHeading = math::NormalizeAngle(sPose.GetHeading());
    Pose2 worldSensorOffset = Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
                                    offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
                                    offsetHeading);

    return sPose - worldSensorOffset;
  }

  /**
   * Gets the bounding box of this scan
   * @return bounding box of this scan
   */
  inline const BoundingBox2 & GetBoundingBox() const
  {
    std::shared_lock<std::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      std::unique_lock<std::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return m_BoundingBox;
  }

  inline void SetBoundingBox(BoundingBox2 & bbox)
  {
    m_BoundingBox = bbox;
  }

  /**
   * Get point readings in local coordinates
   */
  inline const PointVectorDouble & GetPointReadings(kt_bool wantFiltered = false) const
  {
    std::shared_lock<std::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      std::unique_lock<std::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    if (wantFiltered == true) {
      return m_PointReadings;
    } else {
      return m_UnfilteredPointReadings;
    }
  }

  inline void SetPointReadings(PointVectorDouble & points, kt_bool setFiltered = false)
  {
    if (setFiltered) {
      m_PointReadings = points;
    } else {
      m_UnfilteredPointReadings = points;
    }
  }

private:
  /**
   * Compute point readings based on range readings
   * Only range readings within [minimum range; range threshold] are returned
   */
  virtual void Update()
  {
    LaserRangeFinder * pLaserRangeFinder = GetLaserRangeFinder();

    if (pLaserRangeFinder != NULL) {
      m_PointReadings.clear();
      m_UnfilteredPointReadings.clear();

      kt_double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
      kt_double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
      kt_double angularResolution = pLaserRangeFinder->GetAngularResolution();
      Pose2 scanPose = GetSensorPose();

      // compute point readings
      Vector2<kt_double> rangePointsSum;
      kt_int32u beamNum = 0;
      for (kt_int32u i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings(); i++, beamNum++) {
        kt_double rangeReading = GetRangeReadings()[i];
        if (!math::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(), rangeThreshold)) {
          kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

          Vector2<kt_double> point;
          point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
          point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

          m_UnfilteredPointReadings.push_back(point);
          continue;
        }

        kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

        Vector2<kt_double> point;
        point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
        point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

        m_PointReadings.push_back(point);
        m_UnfilteredPointReadings.push_back(point);

        rangePointsSum += point;
      }

      // compute barycenter
      kt_double nPoints = static_cast<kt_double>(m_PointReadings.size());
      if (nPoints != 0.0) {
        Vector2<kt_double> averagePosition = Vector2<kt_double>(rangePointsSum / nPoints);
        m_BarycenterPose = Pose2(averagePosition, 0.0);
      } else {
        m_BarycenterPose = scanPose;
      }

      // calculate bounding box of scan
      m_BoundingBox = BoundingBox2();
      m_BoundingBox.Add(scanPose.GetPosition());
      forEach(PointVectorDouble, &m_PointReadings)
      {
        m_BoundingBox.Add(*iter);
      }
    }

    m_IsDirty = false;
  }

  /**
   * Serialization: class LocalizedRangeScan
   */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_OdometricPose);
    ar & BOOST_SERIALIZATION_NVP(m_CorrectedPose);
    ar & BOOST_SERIALIZATION_NVP(m_BarycenterPose);
    ar & BOOST_SERIALIZATION_NVP(m_PointReadings);
    ar & BOOST_SERIALIZATION_NVP(m_UnfilteredPointReadings);
    ar & BOOST_SERIALIZATION_NVP(m_BoundingBox);
    ar & BOOST_SERIALIZATION_NVP(m_IsDirty);
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(LaserRangeScan);
  }

private:
  LocalizedRangeScan(const LocalizedRangeScan &);
  const LocalizedRangeScan & operator=(const LocalizedRangeScan &);

private:
  /**
   * Odometric pose of robot
   */
  Pose2 m_OdometricPose;

  /**
   * Corrected pose of robot calculated by mapper (or localizer)
   */
  Pose2 m_CorrectedPose;

protected:
  /**
   * Average of all the point readings
   */
  Pose2 m_BarycenterPose;

  /**
   * Vector of point readings
   */
  PointVectorDouble m_PointReadings;

  /**
   * Vector of unfiltered point readings
   */
  PointVectorDouble m_UnfilteredPointReadings;

  /**
   * Bounding box of localized range scan
   */
  BoundingBox2 m_BoundingBox;

  /**
   * Internal flag used to update point readings, barycenter and bounding box
   */
  kt_bool m_IsDirty;
};    // LocalizedRangeScan

/**
 * Type declaration of LocalizedRangeScan vector
 */
typedef std::vector<LocalizedRangeScan *> LocalizedRangeScanVector;
typedef std::map<int, LocalizedRangeScan *> LocalizedRangeScanMap;
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The LocalizedRangeScanWithPoints is an extension of the LocalizedRangeScan with precomputed points.
 */
class LocalizedRangeScanWithPoints : public LocalizedRangeScan
{
public:
  // @cond EXCLUDE
  KARTO_Object(LocalizedRangeScanWithPoints)
  // @endcond

public:
  /**
   * Constructs a range scan from the given range finder with the given readings. Precomptued points should be
   * in the robot frame.
   */
  LocalizedRangeScanWithPoints(
    const Name & rSensorName, const RangeReadingsVector & rReadings,
    const PointVectorDouble & rPoints)
  : m_Points(rPoints),
    LocalizedRangeScan(rSensorName, rReadings)
  {
  }

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScanWithPoints()
  {
  }

private:
  /**
   * Update the points based on the latest sensor pose.
   */
  void Update()
  {
    m_PointReadings.clear();
    m_UnfilteredPointReadings.clear();

    Pose2 scanPose = GetSensorPose();
    Pose2 robotPose = GetCorrectedPose();

    // update point readings
    Vector2<kt_double> rangePointsSum;
    for (kt_int32u i = 0; i < m_Points.size(); i++) {
      // check if this has a NaN
      if (!std::isfinite(m_Points[i].GetX()) || !std::isfinite(m_Points[i].GetY())) {
        Vector2<kt_double> point(m_Points[i].GetX(), m_Points[i].GetY());
        m_UnfilteredPointReadings.push_back(point);

        continue;
      }

      // transform into world coords
      Pose2 pointPose(m_Points[i].GetX(), m_Points[i].GetY(), 0);
      Pose2 result = Transform(robotPose).TransformPose(pointPose);
      Vector2<kt_double> point(result.GetX(), result.GetY());

      m_PointReadings.push_back(point);
      m_UnfilteredPointReadings.push_back(point);

      rangePointsSum += point;
    }

    // compute barycenter
    kt_double nPoints = static_cast<kt_double>(m_PointReadings.size());
    if (nPoints != 0.0) {
      Vector2<kt_double> averagePosition = Vector2<kt_double>(rangePointsSum / nPoints);
      m_BarycenterPose = Pose2(averagePosition, 0.0);
    } else {
      m_BarycenterPose = scanPose;
    }

    // calculate bounding box of scan
    m_BoundingBox = BoundingBox2();
    m_BoundingBox.Add(scanPose.GetPosition());
    forEach(PointVectorDouble, &m_PointReadings)
    {
      m_BoundingBox.Add(*iter);
    }

    m_IsDirty = false;
  }

private:
  LocalizedRangeScanWithPoints(const LocalizedRangeScanWithPoints &);
  const LocalizedRangeScanWithPoints & operator=(const LocalizedRangeScanWithPoints &);

private:
  const PointVectorDouble m_Points;
};    // LocalizedRangeScanWithPoints

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class OccupancyGrid;

class KARTO_EXPORT CellUpdater : public Functor
{
public:
  CellUpdater(OccupancyGrid * pGrid)  // NOLINT
  : m_pOccupancyGrid(pGrid)
  {
  }

  /**
   * Updates the cell at the given index based on the grid's hits and pass counters
   * @param index
   */
  virtual void operator()(kt_int32u index);

private:
  OccupancyGrid * m_pOccupancyGrid;
};    // CellUpdater

/**
 * Occupancy grid definition. See GridStates for possible grid values.
 */
class OccupancyGrid : public Grid<kt_int8u>
{
  friend class CellUpdater;
  friend class IncrementalOccupancyGrid;

public:
  /**
   * Constructs an occupancy grid of given size
   * @param width
   * @param height
   * @param rOffset
   * @param resolution
   */
  OccupancyGrid(
    kt_int32s width, kt_int32s height, const Vector2<kt_double> & rOffset,
    kt_double resolution)
  : Grid<kt_int8u>(width, height),
    m_pCellPassCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution)),
    m_pCellHitsCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution)),
    m_pCellUpdater(NULL)
  {
    m_pCellUpdater = new CellUpdater(this);

    if (karto::math::DoubleEqual(resolution, 0.0)) {
      throw Exception("Resolution cannot be 0");
    }

    m_pMinPassThrough = new Parameter<kt_int32u>("MinPassThrough", 2);
    m_pOccupancyThreshold = new Parameter<kt_double>("OccupancyThreshold", 0.1);

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(rOffset);
  }

  /**
   * Destructor
   */
  virtual ~OccupancyGrid()
  {
    delete m_pCellUpdater;

    delete m_pCellPassCnt;
    delete m_pCellHitsCnt;

    delete m_pMinPassThrough;
    delete m_pOccupancyThreshold;
  }

public:
  /**
   * Create an occupancy grid from the given scans using the given resolution
   * @param rScans
   * @param resolution
   */
  static OccupancyGrid * CreateFromScans(
    const LocalizedRangeScanVector & rScans,
    kt_double resolution)
  {
    if (rScans.empty()) {
      return NULL;
    }

    kt_int32s width, height;
    Vector2<kt_double> offset;
    ComputeDimensions(rScans, resolution, width, height, offset);
    OccupancyGrid * pOccupancyGrid = new OccupancyGrid(width, height, offset, resolution);
    pOccupancyGrid->CreateFromScans(rScans);

    return pOccupancyGrid;
  }

  /**
   * Make a clone
   * @return occupancy grid clone
   */
  OccupancyGrid * Clone() const
  {
    OccupancyGrid * pOccupancyGrid = new OccupancyGrid(GetWidth(),
        GetHeight(),
        GetCoordinateConverter()->GetOffset(),
        1.0 / GetCoordinateConverter()->GetScale());
    memcpy(pOccupancyGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    pOccupancyGrid->GetCoordinateConverter()->SetSize(GetCoordinateConverter()->GetSize());
    pOccupancyGrid->m_pCellPassCnt = m_pCellPassCnt->Clone();
    pOccupancyGrid->m_pCellHitsCnt = m_pCellHitsCnt->Clone();

    return pOccupancyGrid;
  }

  /**
   * Check if grid point is free
   * @param rPose
   * @return whether the cell at the given point is free space
   */
  virtual kt_bool IsFree(const Vector2<kt_int32s> & rPose) const
  {
    kt_int8u * pOffsets = reinterpret_cast<kt_int8u *>(GetDataPointer(rPose));
    if (*pOffsets == GridStates_Free) {
      return true;
    }

    return false;
  }

  /**
   * Casts a ray from the given point (up to the given max range)
   * and returns the distance to the closest obstacle
   * @param rPose2
   * @param maxRange
   * @return distance to closest obstacle
   */
  virtual kt_double RayCast(const Pose2 & rPose2, kt_double maxRange) const
  {
    double scale = GetCoordinateConverter()->GetScale();

    kt_double x = rPose2.GetX();
    kt_double y = rPose2.GetY();
    kt_double theta = rPose2.GetHeading();

    kt_double sinTheta = sin(theta);
    kt_double cosTheta = cos(theta);

    kt_double xStop = x + maxRange * cosTheta;
    kt_double xSteps = 1 + fabs(xStop - x) * scale;

    kt_double yStop = y + maxRange * sinTheta;
    kt_double ySteps = 1 + fabs(yStop - y) * scale;

    kt_double steps = math::Maximum(xSteps, ySteps);
    kt_double delta = maxRange / steps;
    kt_double distance = delta;

    for (kt_int32u i = 1; i < steps; i++) {
      kt_double x1 = x + distance * cosTheta;
      kt_double y1 = y + distance * sinTheta;

      Vector2<kt_int32s> gridIndex = WorldToGrid(Vector2<kt_double>(x1, y1));
      if (IsValidGridIndex(gridIndex) && IsFree(gridIndex)) {
        distance = (i + 1) * delta;
      } else {
        break;
      }
    }

    return (distance < maxRange) ? distance : maxRange;
  }

  /**
   * Sets the minimum number of beams that must pass through a cell before it
   * will be considered to be occupied or unoccupied.
   * This prevents stray beams from messing up the map.
   */
  void SetMinPassThrough(kt_int32u count)
  {
    m_pMinPassThrough->SetValue(count);
  }

  /**
   * Sets the minimum ratio of beams hitting cell to beams passing through
   * cell for cell to be marked as occupied.
   */
  void SetOccupancyThreshold(kt_double thresh)
  {
    m_pOccupancyThreshold->SetValue(thresh);
  }

protected:
  /**
   * Get cell hit grid
   * @return Grid<kt_int32u>*
   */
  virtual Grid<kt_int32u> * GetCellHitsCounts()
  {
    return m_pCellHitsCnt;
  }

  /**
   * Get cell pass grid
   * @return Grid<kt_int32u>*
   */
  virtual Grid<kt_int32u> * GetCellPassCounts()
  {
    return m_pCellPassCnt;
  }

protected:
  /**
   * Calculate grid dimensions from localized range scans
   * @param rScans
   * @param resolution
   * @param rWidth
   * @param rHeight
   * @param rOffset
   */
  static void ComputeDimensions(
    const LocalizedRangeScanVector & rScans,
    kt_double resolution,
    kt_int32s & rWidth,
    kt_int32s & rHeight,
    Vector2<kt_double> & rOffset)
  {
    BoundingBox2 boundingBox;

    const_forEach(LocalizedRangeScanVector, &rScans)
    {
      if (*iter == nullptr) {
        continue;
      }

      boundingBox.Add((*iter)->GetBoundingBox());
    }

    kt_double scale = 1.0 / resolution;
    Size2<kt_double> size = boundingBox.GetSize();

    rWidth = static_cast<kt_int32s>(math::Round(size.GetWidth() * scale));
    rHeight = static_cast<kt_int32s>(math::Round(size.GetHeight() * scale));
    rOffset = boundingBox.GetMinimum();
  }

  /**
   * Create grid using scans
   * @param rScans
   */
  virtual void CreateFromScans(const LocalizedRangeScanVector & rScans)
  {
    m_pCellPassCnt->Resize(GetWidth(), GetHeight());
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    m_pCellHitsCnt->Resize(GetWidth(), GetHeight());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    const_forEach(LocalizedRangeScanVector, &rScans)
    {
      if (*iter == nullptr) {
        continue;
      }

      LocalizedRangeScan * pScan = *iter;
      AddScan(pScan);
    }

    Update();
  }

  /**
   * Adds the scan's information to this grid's counters (optionally
   * update the grid's cells' occupancy status)
   * @param pScan
   * @param doUpdate whether to update the grid's cell's occupancy status
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual kt_bool AddScan(LocalizedRangeScan * pScan, kt_bool doUpdate = false)
  {
    LaserRangeFinder * laserRangeFinder = pScan->GetLaserRangeFinder();
    kt_double rangeThreshold = laserRangeFinder->GetRangeThreshold();
    kt_double maxRange = laserRangeFinder->GetMaximumRange();
    kt_double minRange = laserRangeFinder->GetMinimumRange();

    Vector2<kt_double> scanPosition = pScan->GetSensorPose().GetPosition();
    // get scan point readings
    const PointVectorDouble & rPointReadings = pScan->GetPointReadings(false);

    kt_bool isAllInMap = true;

    // draw lines from scan position to all point readings
    int pointIndex = 0;
    const_forEachAs(PointVectorDouble, &rPointReadings, pointsIter)
    {
      Vector2<kt_double> point = *pointsIter;
      kt_double rangeReading = pScan->GetRangeReadings()[pointIndex];
      kt_bool isEndPointValid = rangeReading < (rangeThreshold - KT_TOLERANCE);

      if (rangeReading <= minRange || rangeReading >= maxRange || std::isnan(rangeReading)) {
        // ignore these readings
        pointIndex++;
        continue;
      } else if (rangeReading >= rangeThreshold) {
        // trace up to range reading
        kt_double ratio = rangeThreshold / rangeReading;
        kt_double dx = point.GetX() - scanPosition.GetX();
        kt_double dy = point.GetY() - scanPosition.GetY();
        point.SetX(scanPosition.GetX() + ratio * dx);
        point.SetY(scanPosition.GetY() + ratio * dy);
      }

      kt_bool isInMap = RayTrace(scanPosition, point, isEndPointValid, doUpdate);
      if (!isInMap) {
        isAllInMap = false;
      }

      pointIndex++;
    }

    return isAllInMap;
  }

  /**
   * Traces a beam from the start position to the end position marking
   * the bookkeeping arrays accordingly.
   * @param rWorldFrom start position of beam
   * @param rWorldTo end position of beam
   * @param isEndPointValid is the reading within the range threshold?
   * @param doUpdate whether to update the cells' occupancy status immediately
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual kt_bool RayTrace(
    const Vector2<kt_double> & rWorldFrom,
    const Vector2<kt_double> & rWorldTo,
    kt_bool isEndPointValid,
    kt_bool doUpdate = false)
  {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    Vector2<kt_int32s> gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
    Vector2<kt_int32s> gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);

    CellUpdater * pCellUpdater = doUpdate ? m_pCellUpdater : NULL;
    m_pCellPassCnt->TraceLine(gridFrom.GetX(), gridFrom.GetY(), gridTo.GetX(),
      gridTo.GetY(), pCellUpdater);

    // for the end point
    if (isEndPointValid) {
      if (m_pCellPassCnt->IsValidGridIndex(gridTo)) {
        kt_int32s index = m_pCellPassCnt->GridIndex(gridTo, false);

        kt_int32u * pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
        kt_int32u * pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

        // increment cell pass through and hit count
        pCellPassCntPtr[index]++;
        pCellHitCntPtr[index]++;

        if (doUpdate) {
          (*m_pCellUpdater)(index);
        }
      }
    }

    return m_pCellPassCnt->IsValidGridIndex(gridTo);
  }

  /**
   * Updates a single cell's value based on the given counters
   * @param pCell
   * @param cellPassCnt
   * @param cellHitCnt
   */
  virtual void UpdateCell(kt_int8u * pCell, kt_int32u cellPassCnt, kt_int32u cellHitCnt)
  {
    if (cellPassCnt > m_pMinPassThrough->GetValue()) {
      kt_double hitRatio = static_cast<kt_double>(cellHitCnt) / static_cast<kt_double>(cellPassCnt);

      if (hitRatio > m_pOccupancyThreshold->GetValue()) {
        *pCell = GridStates_Occupied;
      } else {
        *pCell = GridStates_Free;
      }
    }
  }

  /**
   * Update the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
   */
  virtual void Update()
  {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    // clear grid
    Clear();

    // set occupancy status of cells
    kt_int8u * pDataPtr = GetDataPointer();
    kt_int32u * pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
    kt_int32u * pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

    kt_int32u nBytes = GetDataSize();
    for (kt_int32u i = 0; i < nBytes; i++, pDataPtr++, pCellPassCntPtr++, pCellHitCntPtr++) {
      UpdateCell(pDataPtr, *pCellPassCntPtr, *pCellHitCntPtr);
    }
  }

  /**
   * Resizes the grid (deletes all old data)
   * @param width
   * @param height
   */
  virtual void Resize(kt_int32s width, kt_int32s height)
  {
    Grid<kt_int8u>::Resize(width, height);
    m_pCellPassCnt->Resize(width, height);
    m_pCellHitsCnt->Resize(width, height);
  }

protected:
  /**
   * Counters of number of times a beam passed through a cell
   */
  Grid<kt_int32u> * m_pCellPassCnt;

  /**
   * Counters of number of times a beam ended at a cell
   */
  Grid<kt_int32u> * m_pCellHitsCnt;

private:
  /**
   * Restrict the copy constructor
   */
  OccupancyGrid(const OccupancyGrid &);

  /**
   * Restrict the assignment operator
   */
  const OccupancyGrid & operator=(const OccupancyGrid &);

private:
  CellUpdater * m_pCellUpdater;

  ////////////////////////////////////////////////////////////
  // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
  // then not many beams will hit the cell!

  // Number of beams that must pass through a cell before it will be considered to be occupied
  // or unoccupied.  This prevents stray beams from messing up the map.
  Parameter<kt_int32u> * m_pMinPassThrough;

  // Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied
  Parameter<kt_double> * m_pOccupancyThreshold;
};    // OccupancyGrid

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Dataset info
 * Contains title, author and other information about the dataset
 */
class DatasetInfo : public Object
{
public:
  // @cond EXCLUDE
  KARTO_Object(DatasetInfo)
  // @endcond

public:
  DatasetInfo()
  : Object()
  {
    m_pTitle = new Parameter<std::string>("Title", "", GetParameterManager());
    m_pAuthor = new Parameter<std::string>("Author", "", GetParameterManager());
    m_pDescription = new Parameter<std::string>("Description", "", GetParameterManager());
    m_pCopyright = new Parameter<std::string>("Copyright", "", GetParameterManager());
  }

  virtual ~DatasetInfo()
  {
  }

public:
  /**
   * Dataset title
   */
  const std::string & GetTitle() const
  {
    return m_pTitle->GetValue();
  }

  /**
   * Dataset author(s)
   */
  const std::string & GetAuthor() const
  {
    return m_pAuthor->GetValue();
  }

  /**
   * Dataset description
   */
  const std::string & GetDescription() const
  {
    return m_pDescription->GetValue();
  }

  /**
   * Dataset copyrights
   */
  const std::string & GetCopyright() const
  {
    return m_pCopyright->GetValue();
  }

  /**
   * Serialization: class DatasetInfo
   */

private:
  DatasetInfo(const DatasetInfo &);
  const DatasetInfo & operator=(const DatasetInfo &);

private:
  Parameter<std::string> * m_pTitle;
  Parameter<std::string> * m_pAuthor;
  Parameter<std::string> * m_pDescription;
  Parameter<std::string> * m_pCopyright;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Object);
    ar & BOOST_SERIALIZATION_NVP(*m_pTitle);
    ar & BOOST_SERIALIZATION_NVP(*m_pAuthor);
    ar & BOOST_SERIALIZATION_NVP(*m_pDescription);
    ar & BOOST_SERIALIZATION_NVP(*m_pCopyright);
  }
};    // class DatasetInfo

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Karto dataset. Stores LaserRangeFinders and LocalizedRangeScans and manages memory of allocated LaserRangeFinders
 * and LocalizedRangeScans
 */
class Dataset
{
public:
  /**
   * Default constructor
   */
  Dataset()
  : m_pDatasetInfo(NULL)
  {
  }

  /**
   * Destructor
   */
  virtual ~Dataset()
  {
    Clear();
  }

  /**
   * Save dataset to file
   * @param filename
   */
  void SaveToFile(const std::string & filename)
  {
    printf("Save To File\n");
    std::ofstream ofs(filename.c_str());
    boost::archive::binary_oarchive oa(ofs, boost::archive::no_codecvt);
    oa << BOOST_SERIALIZATION_NVP(*this);
  }

  /**
   * Load dataset from file
   * @param filename
   */
  void LoadFromFile(const std::string & filename)
  {
    printf("Load From File\n");
    std::ifstream ifs(filename.c_str());
    boost::archive::binary_iarchive ia(ifs, boost::archive::no_codecvt);  // no second arg?
    ia >> BOOST_SERIALIZATION_NVP(*this);
  }

public:
  /**
   * Adds object to this dataset
   * @param pObject
   */
  void Add(Object * pObject, kt_bool overrideSensorName = false)
  {
    if (pObject != NULL) {
      if (dynamic_cast<Sensor *>(pObject)) {
        Sensor * pSensor = dynamic_cast<Sensor *>(pObject);
        if (pSensor != NULL) {
          m_SensorNameLookup[pSensor->GetName()] = pSensor;
          karto::SensorManager::GetInstance()->RegisterSensor(pSensor, overrideSensorName);
        }

        m_Lasers.push_back(pObject);
      } else if (dynamic_cast<SensorData *>(pObject)) {
        SensorData * pSensorData = dynamic_cast<SensorData *>(pObject);
        m_Data.insert({pSensorData->GetUniqueId(), pSensorData});
      } else if (dynamic_cast<DatasetInfo *>(pObject)) {
        m_pDatasetInfo = dynamic_cast<DatasetInfo *>(pObject);
      } else {
        std::cout << "Did not save object of non-data and non-sensor type" << std::endl;
      }
    }
  }

  /**
   * Get sensor states
   * @return sensor state
   */
  inline const ObjectVector & GetLasers() const
  {
    return m_Lasers;
  }

  /**
   * Get data states
   * @return data state
   */
  inline const DataMap & GetData() const
  {
    return m_Data;
  }

  /**
   * Remove data
   * @param index to remove
   */
  inline void RemoveData(LocalizedRangeScan * scan)
  {
    auto iterator = m_Data.find(scan->GetUniqueId());
    if (iterator != m_Data.end()) {
      delete iterator->second;
      iterator->second = nullptr;
      m_Data.erase(iterator);
    } else {
      std::cout <<
        "Failed to remove data. Pointer to LocalizedRangeScan could not be found in dataset. " <<
        "Most likely different pointer address but same object TODO STEVE." << std::endl;
    }
  }

  /**
   * Get dataset info
   * @return dataset info
   */
  inline DatasetInfo * GetDatasetInfo()
  {
    return m_pDatasetInfo;
  }

  /**
   * Delete all stored data
   */
  virtual void Clear()
  {
    for (std::map<Name, Sensor *>::iterator iter = m_SensorNameLookup.begin();
      iter != m_SensorNameLookup.end(); ++iter)
    {
      karto::SensorManager::GetInstance()->UnregisterSensor(iter->second);
    }

    forEach(ObjectVector, &m_Lasers)
    {
      if (*iter) {
        delete *iter;
        *iter = NULL;
      }
    }

    for (auto iter = m_Data.begin(); iter != m_Data.end(); ++iter) {
      if (iter->second) {
        delete iter->second;
        iter->second = NULL;
      }
    }

    m_Lasers.clear();
    m_Data.clear();

    if (m_pDatasetInfo != NULL) {
      delete m_pDatasetInfo;
      m_pDatasetInfo = NULL;
    }
  }

private:
  std::map<Name, Sensor *> m_SensorNameLookup;
  ObjectVector m_Lasers;
  DataMap m_Data;
  DatasetInfo * m_pDatasetInfo;
  /**
   * Serialization: class Dataset
   */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    std::cout << "**Serializing Dataset**\n";
    std::cout << "Dataset <- m_SensorNameLookup\n";
    ar & BOOST_SERIALIZATION_NVP(m_SensorNameLookup);
    std::cout << "Dataset <- m_Data\n";
    ar & BOOST_SERIALIZATION_NVP(m_Data);
    std::cout << "Dataset <- m_Lasers\n";
    ar & BOOST_SERIALIZATION_NVP(m_Lasers);
    std::cout << "Dataset <- m_pDatasetInfo\n";
    ar & BOOST_SERIALIZATION_NVP(m_pDatasetInfo);
    std::cout << "**Finished serializing Dataset**\n";
  }
};    // Dataset
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Dataset)
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * An array that can be resized as long as the size
 * does not exceed the initial capacity
 */
class LookupArray
{
public:
  /**
   * Constructs lookup array
   */
  LookupArray()
  : m_pArray(NULL),
    m_Capacity(0),
    m_Size(0)
  {
  }

  /**
   * Destructor
   */
  virtual ~LookupArray()
  {
    assert(m_pArray != NULL);

    delete[] m_pArray;
    m_pArray = NULL;
  }

public:
  /**
   * Clear array
   */
  void Clear()
  {
    memset(m_pArray, 0, sizeof(kt_int32s) * m_Capacity);
  }

  /**
   * Gets size of array
   * @return array size
   */
  kt_int32u GetSize() const
  {
    return m_Size;
  }

  /**
   * Sets size of array (resize if not big enough)
   * @param size
   */
  void SetSize(kt_int32u size)
  {
    assert(size != 0);

    if (size > m_Capacity) {
      if (m_pArray != NULL) {
        delete[] m_pArray;
      }
      m_Capacity = size;
      m_pArray = new kt_int32s[m_Capacity];
    }

    m_Size = size;
  }

  /**
   * Gets reference to value at given index
   * @param index
   * @return reference to value at index
   */
  inline kt_int32s & operator[](kt_int32u index)
  {
    assert(index < m_Size);

    return m_pArray[index];
  }

  /**
   * Gets value at given index
   * @param index
   * @return value at index
   */
  inline kt_int32s operator[](kt_int32u index) const
  {
    assert(index < m_Size);

    return m_pArray[index];
  }

  /**
   * Gets array pointer
   * @return array pointer
   */
  inline kt_int32s * GetArrayPointer()
  {
    return m_pArray;
  }

  /**
   * Gets array pointer
   * @return array pointer
   */
  inline kt_int32s * GetArrayPointer() const
  {
    return m_pArray;
  }

private:
  kt_int32s * m_pArray;
  kt_int32u m_Capacity;
  kt_int32u m_Size;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Capacity);
    ar & BOOST_SERIALIZATION_NVP(m_Size);
    if (Archive::is_loading::value) {
      m_pArray = new kt_int32s[m_Capacity];
    }
    ar & boost::serialization::make_array<kt_int32s>(m_pArray, m_Capacity);
  }
};    // LookupArray

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Create lookup tables for point readings at varying angles in grid.
 * For each angle, grid indexes are calculated for each range reading.
 * This is to speed up finding best angle/position for a localized range scan
 *
 * Used heavily in mapper and localizer.
 *
 * In the localizer, this is a huge speed up for calculating possible position.  For each particle,
 * a probability is calculated.  The range scan is the same, but all grid indexes at all possible angles are
 * calculated.  So when calculating the particle probability at a specific angle, the index table is used
 * to look up probability in probability grid!
 *
 */
template<typename T>
class GridIndexLookup
{
public:
  /**
   * Construct a GridIndexLookup with a grid
   * @param pGrid
   */
  GridIndexLookup()
  {
  }

  GridIndexLookup(Grid<T> * pGrid)  // NOLINT
  : m_pGrid(pGrid),
    m_Capacity(0),
    m_Size(0),
    m_ppLookupArray(NULL)
  {
  }

  /**
   * Destructor
   */
  virtual ~GridIndexLookup()
  {
    DestroyArrays();
  }

public:
  /**
   * Gets the lookup array for a particular angle index
   * @param index
   * @return lookup array
   */
  const LookupArray * GetLookupArray(kt_int32u index) const
  {
    assert(math::IsUpTo(index, m_Size));

    return m_ppLookupArray[index];
  }

  /**
   * Get angles
   * @return std::vector<kt_double>& angles
   */
  const std::vector<kt_double> & GetAngles() const
  {
    return m_Angles;
  }

  /**
   * Compute lookup table of the points of the given scan for the given angular space
   * @param pScan the scan
   * @param angleCenter
   * @param angleOffset computes lookup arrays for the angles within this offset around angleStart
   * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
   */
  void ComputeOffsets(
    LocalizedRangeScan * pScan,
    kt_double angleCenter,
    kt_double angleOffset,
    kt_double angleResolution)
  {
    assert(angleOffset != 0.0);
    assert(angleResolution != 0.0);

    kt_int32u nAngles =
      static_cast<kt_int32u>(math::Round(angleOffset * 2.0 / angleResolution) + 1);
    SetSize(nAngles);

    //////////////////////////////////////////////////////
    // convert points into local coordinates of scan pose

    const PointVectorDouble & rPointReadings = pScan->GetPointReadings();

    // compute transform to scan pose
    Transform transform(pScan->GetSensorPose());

    Pose2Vector localPoints;
    const_forEach(PointVectorDouble, &rPointReadings)
    {
      // do inverse transform to get points in local coordinates
      Pose2 vec = transform.InverseTransformPose(Pose2(*iter, 0.0));
      localPoints.push_back(vec);
    }

    //////////////////////////////////////////////////////
    // create lookup array for different angles
    kt_double angle = 0.0;
    kt_double startAngle = angleCenter - angleOffset;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++) {
      angle = startAngle + angleIndex * angleResolution;
      ComputeOffsets(angleIndex, angle, localPoints, pScan);
    }
    // assert(math::DoubleEqual(angle, angleCenter + angleOffset));
  }

private:
  /**
   * Compute lookup value of points for given angle
   * @param angleIndex
   * @param angle
   * @param rLocalPoints
   */
  void ComputeOffsets(
    kt_int32u angleIndex, kt_double angle, const Pose2Vector & rLocalPoints,
    LocalizedRangeScan * pScan)
  {
    m_ppLookupArray[angleIndex]->SetSize(static_cast<kt_int32u>(rLocalPoints.size()));
    m_Angles.at(angleIndex) = angle;

    // set up point array by computing relative offsets to points readings
    // when rotated by given angle

    const Vector2<kt_double> & rGridOffset = m_pGrid->GetCoordinateConverter()->GetOffset();

    kt_double cosine = cos(angle);
    kt_double sine = sin(angle);

    kt_int32u readingIndex = 0;

    kt_int32s * pAngleIndexPointer = m_ppLookupArray[angleIndex]->GetArrayPointer();

    kt_double maxRange = pScan->GetLaserRangeFinder()->GetMaximumRange();

    const_forEach(Pose2Vector, &rLocalPoints)
    {
      const Vector2<kt_double> & rPosition = iter->GetPosition();

      if (std::isnan(pScan->GetRangeReadings()[readingIndex]) ||
        std::isinf(pScan->GetRangeReadings()[readingIndex]))
      {
        pAngleIndexPointer[readingIndex] = INVALID_SCAN;
        readingIndex++;
        continue;
      }


      // counterclockwise rotation and that rotation is about the origin (0, 0).
      Vector2<kt_double> offset;
      offset.SetX(cosine * rPosition.GetX() - sine * rPosition.GetY());
      offset.SetY(sine * rPosition.GetX() + cosine * rPosition.GetY());

      // have to compensate for the grid offset when getting the grid index
      Vector2<kt_int32s> gridPoint = m_pGrid->WorldToGrid(offset + rGridOffset);

      // use base GridIndex to ignore ROI
      kt_int32s lookupIndex = m_pGrid->Grid<T>::GridIndex(gridPoint, false);

      pAngleIndexPointer[readingIndex] = lookupIndex;

      readingIndex++;
    }
    assert(readingIndex == rLocalPoints.size());
  }

  /**
   * Sets size of lookup table (resize if not big enough)
   * @param size
   */
  void SetSize(kt_int32u size)
  {
    assert(size != 0);

    if (size > m_Capacity) {
      if (m_ppLookupArray != NULL) {
        DestroyArrays();
      }

      m_Capacity = size;
      m_ppLookupArray = new LookupArray * [m_Capacity];
      for (kt_int32u i = 0; i < m_Capacity; i++) {
        m_ppLookupArray[i] = new LookupArray();
      }
    }

    m_Size = size;

    m_Angles.resize(size);
  }

  /**
   * Delete the arrays
   */
  void DestroyArrays()
  {
    if (m_ppLookupArray) {
      for (kt_int32u i = 0; i < m_Capacity; i++) {
        delete m_ppLookupArray[i];
      }
    }
    if (m_ppLookupArray) {
      delete[] m_ppLookupArray;
      m_ppLookupArray = NULL;
    }
  }

private:
  Grid<T> * m_pGrid;

  kt_int32u m_Capacity;
  kt_int32u m_Size;

  LookupArray ** m_ppLookupArray;

  // for sanity check
  std::vector<kt_double> m_Angles;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_pGrid);
    ar & BOOST_SERIALIZATION_NVP(m_Capacity);
    ar & BOOST_SERIALIZATION_NVP(m_Size);
    ar & BOOST_SERIALIZATION_NVP(m_Angles);
    if (Archive::is_loading::value) {
      m_ppLookupArray = new LookupArray * [m_Capacity];
      for (kt_int32u i = 0; i < m_Capacity; i++) {
        m_ppLookupArray[i] = new LookupArray();
      }
    }
    ar & boost::serialization::make_array<LookupArray *>(m_ppLookupArray, m_Capacity);
  }
};            // class GridIndexLookup

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

inline Pose2::Pose2(const Pose3 & rPose)
: m_Position(rPose.GetPosition().GetX(), rPose.GetPosition().GetY())
{
  kt_double t1, t2;

  // calculates heading from orientation
  rPose.GetOrientation().ToEulerAngles(m_Heading, t1, t2);
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// @cond EXCLUDE

template<typename T>
inline void Object::SetParameter(const std::string & rName, T value)
{
  AbstractParameter * pParameter = GetParameter(rName);
  if (pParameter != NULL) {
    std::stringstream stream;
    stream << value;
    pParameter->SetValueFromString(stream.str());
  } else {
    throw Exception("Parameter does not exist:  " + rName);
  }
}

template<>
inline void Object::SetParameter(const std::string & rName, kt_bool value)
{
  AbstractParameter * pParameter = GetParameter(rName);
  if (pParameter != NULL) {
    pParameter->SetValueFromString(value ? "true" : "false");
  } else {
    throw Exception("Parameter does not exist:  " + rName);
  }
}

// @endcond

/*@}*/
}   // namespace karto

BOOST_CLASS_EXPORT_KEY(karto::NonCopyable);
BOOST_CLASS_EXPORT_KEY(karto::Object);
BOOST_CLASS_EXPORT_KEY(karto::Sensor);
BOOST_CLASS_EXPORT_KEY(karto::Name);
BOOST_CLASS_EXPORT_KEY(karto::SensorData);
BOOST_CLASS_EXPORT_KEY(karto::LocalizedRangeScan);
BOOST_CLASS_EXPORT_KEY(karto::LaserRangeScan);
BOOST_CLASS_EXPORT_KEY(karto::LaserRangeFinder);
BOOST_CLASS_EXPORT_KEY(karto::CustomData);
BOOST_CLASS_EXPORT_KEY(karto::Module);
BOOST_CLASS_EXPORT_KEY(karto::Rectangle2<kt_double>);
BOOST_CLASS_EXPORT_KEY(karto::CoordinateConverter);
BOOST_CLASS_EXPORT_KEY(karto::Dataset);
BOOST_CLASS_EXPORT_KEY(karto::SensorManager);
BOOST_CLASS_EXPORT_KEY(karto::Size2<kt_double>);
BOOST_CLASS_EXPORT_KEY(karto::GridIndexLookup<kt_int8u>);
BOOST_CLASS_EXPORT_KEY(karto::LookupArray);
BOOST_CLASS_EXPORT_KEY(karto::AbstractParameter);
BOOST_CLASS_EXPORT_KEY(karto::ParameterEnum);
BOOST_CLASS_EXPORT_KEY(karto::Parameters);
BOOST_CLASS_EXPORT_KEY(karto::ParameterManager);
BOOST_CLASS_EXPORT_KEY(karto::Parameter<kt_double>);
BOOST_CLASS_EXPORT_KEY(karto::Parameter<karto::Pose2>);
BOOST_CLASS_EXPORT_KEY(karto::Parameter<kt_bool>);
BOOST_CLASS_EXPORT_KEY(karto::Parameter<kt_int32u>);
BOOST_CLASS_EXPORT_KEY(karto::Parameter<kt_int32s>);
BOOST_CLASS_EXPORT_KEY(karto::Parameter<std::string>);

#endif  // KARTO_SDK__KARTO_H_
