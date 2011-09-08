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

int processFlightControlQuadPlusManual(pilota_t *pilota_quad);
int processFlightControlQuadPlus(pilota_t *pilota_quad);
int processFlightControlQuadAuto(pilota_t *pilota_quad);

int processFlightControlQuadXManual(pilota_t *pilota_quad);
int processFlightControlQuadX(pilota_t *pilota_quad);
int processFlightControlQuadXAuto(pilota_t *pilota_quad);


//GUIDED
void processAttitude(pilota_t *pilota_quad);
void processHeading(pilota_t *pilota_quad);
void processThrottle(pilota_t *pilota_quad);

//AUTO
void processAltitude(pilota_t *pilota_quad);
void processPosition(pilota_t *pilota_quad);

float get_bearing(pilota_location_t *start,pilota_location_t *end);			//direzione
float get_distance(pilota_location_t *start,pilota_location_t *end);		//distanza
float get_alt_distance(pilota_location_t *start,pilota_location_t *end);	//altezza





int16_t angle_to_10000(float angle);

#endif /* QUAD_H_ */
