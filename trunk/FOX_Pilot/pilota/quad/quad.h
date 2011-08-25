/*
 * quad.h
 *
 *  Created on: 26/apr/2011
 *      Author: vito
 */

#ifndef QUAD_H_
#define QUAD_H_

#include <config.h>
#include <defines.h>

#include <pilota.h>

//QUAD
#define FR 0	//Front
#define LE 1	//Left
#define RE 2	//Rear
#define RI 3	//Rigth

//QUADX
//FR			//Front Rigth
#define FL LE	//Front Left
#define RL RE	//Rear Left
#define RR RI	//Rear Rigth

int processFlightControlQuadPlus(pilota_t *pilota_quad);
int processFlightControlQuadX(pilota_t *pilota_quad);

#endif /* QUAD_H_ */
