/*
 * filtri.c
 *
 *  Created on: 26/apr/2011
 *      Author: vito
 */

#include <math/filtri.h>

int filterSmooth(int currentData, int previousData, int smoothFactor) {
  if (smoothFactor != 100) // only apply time compensated filter if smoothFactor is applied
    return ((previousData * (100 - smoothFactor) + (currentData * smoothFactor))/100);
  else
    return currentData; // if smoothFactor == 100, do not calculate, just bypass!
}
