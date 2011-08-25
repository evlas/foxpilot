/*
 * filtri.h
 *
 *  Created on: 26/apr/2011
 *      Author: vito
 */

#ifndef FILTRI_H_
#define FILTRI_H_

//filtro passabasso a un passo
int filterSmooth(int currentData, int previousData, int smoothFactor);

#endif /* FILTRI_H_ */
