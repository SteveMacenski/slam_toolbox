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

#ifndef KARTO_SDK__TYPES_H_
#define KARTO_SDK__TYPES_H_

#include <cstddef>

/**
 * Karto type definition
 * Ensures platform independent types for windows, linux and mac
 */
#if defined(_MSC_VER)

typedef signed __int8 kt_int8s;
typedef unsigned __int8 kt_int8u;

typedef signed __int16 kt_int16s;
typedef unsigned __int16 kt_int16u;

typedef signed __int32 kt_int32s;
typedef unsigned __int32 kt_int32u;

typedef signed __int64 kt_int64s;
typedef unsigned __int64 kt_int64u;

#else

  #include <stdint.h>

typedef int8_t kt_int8s;
typedef uint8_t kt_int8u;

typedef int16_t kt_int16s;
typedef uint16_t kt_int16u;

typedef int32_t kt_int32s;
typedef uint32_t kt_int32u;

#if defined(__LP64__)
typedef signed long kt_int64s;  // NOLINT
typedef unsigned long kt_int64u;  // NOLINT
#else
typedef signed long long kt_int64s;  // NOLINT
typedef unsigned long long kt_int64u;  // NOLINT
#endif

#endif

typedef bool kt_bool;
typedef char kt_char;
typedef float kt_float;
typedef double kt_double;

#endif  // KARTO_SDK__TYPES_H_
