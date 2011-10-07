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
int processFlightControlQuadPlusGuided(pilota_t *pilota_quad);
int processFlightControlQuadPlusAuto(pilota_t *pilota_quad);

int processFlightControlQuadXManual(pilota_t *pilota_quad);
int processFlightControlQuadXGuided(pilota_t *pilota_quad);
int processFlightControlQuadXAuto(pilota_t *pilota_quad);

int guided_fly_handler(pilota_t *pilota_quad);
int	auto_fly_handler(pilota_t *pilota_quad);

void processAttitude(pilota_t *pilota_quad, float roll, float pitch);
void processHeading(pilota_t *pilota_quad, float heading);
void processThrottle(pilota_t *pilota_quad, float throttle);
void processAltitude(pilota_t *pilota_quad, float altitude);
void processPosition(pilota_t *pilota_quad);

float get_bearing(pilota_location_t *start,pilota_location_t *end);			//direzione
float get_distance(pilota_location_t *start,pilota_location_t *end);		//distanza
float get_alt_distance(pilota_location_t *start,pilota_location_t *end);	//altezza

//alti in cm
int take_off(pilota_t *pilota_quad, float alti, enum MAV_FRAME frame);
int landing(pilota_t *pilota_quad);

int16_t angle_to_10000(float angle);

#endif /* QUAD_H_ */
