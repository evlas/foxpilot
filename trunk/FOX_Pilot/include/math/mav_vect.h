/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann <mavteam@student.ethz.ch>
  Dominik Honegger <mavteam@student.ethz.ch>
Contributing Authors (in alphabetical order):



(c) 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mavlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mavlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mavlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>

#ifndef MAV_VECT_H_
#define MAV_VECT_H_

typedef struct{
  int32_t x;
  int32_t y;
} int32_vect2;

typedef struct{
  int32_t x;
  int32_t y;
  int32_t z;
} int32_vect3;

typedef struct{
  int16_t x;
  int16_t y;
  int16_t z;
} int16_vect3;

typedef struct{
  uint16_t x;
  uint16_t y;
  uint16_t z;
} uint16_vect3;

typedef struct {
  float x;
  float y;
  float z;
} float_vect3;

typedef struct{
  int32_t w;
  int32_t x;
  int32_t y;
  int32_t z;
} int32_vect4;

static float_vect3 operator_minus (float_vect3 l, float_vect3 r) {
	float_vect3 ret;

	ret.x = l.x - r.x;
	ret.y = l.y - r.y;
	ret.z = l.z - r.z;

	return (ret);
}

static float_vect3 operator_plus (float_vect3 l, float_vect3 r) {
	float_vect3 ret;

	ret.x = l.x + r.x;
	ret.y = l.y + r.y;
	ret.z = l.z + r.z;

	return (ret);
}

static float operator_per(float_vect3 l, float_vect3 r) {
	return (l.x*r.x + l.y*r.y + l.z*r.z);
}

static float_vect3 operator_per_scalar(float l, float_vect3 r) {
	float_vect3 ret;

	ret.x = l * r.x;
	ret.y = l * r.y;
	ret.z = l * r.z;

	return (ret);
}

static float operator_square(float_vect3 l) {
	return (l.x*l.x + l.y*l.y + l.z*l.z);
}

#endif /* MAV_VECT_H_ */

