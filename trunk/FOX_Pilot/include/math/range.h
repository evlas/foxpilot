/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
* @file
*   @brief Min, max and clamp functions
*   @author Lorenz Meier
*/

#ifndef RANGE_H_
#define RANGE_H_

#define clamp(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define min(x, y)  (((x) < (y)) ? (x) : (y))
#define max(x, y)  (((x) > (y)) ? (x) : (y))

#define remap(x, rc_min, rc_max, out_min, out_max) (((x)-(rc_min))*((out_max)-(out_min))/((rc_max)-(rc_min)))+(out_min)
/*
static inline int constrain_int(int x, int a, int b) {
	if (x<a) {
		return(a);
	}
	if (x>b) {
		return(b);
	}
	return(x);
}
static inline float constrain_float(float x, float a, float b) {
	if (x<a) {
		return(a);
	}
	if (x>b) {
		return(b);
	}
	return(x);
}
static inline double constrain_double(double x, double a, double b) {
	if (x<a) {
		return(a);
	}
	if (x>b) {
		return(b);
	}
	return(x);
}

*/

#endif
