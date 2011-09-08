/*
 * pilota.h
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#ifndef PILOTA_H_
#define PILOTA_H_

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include <defines.h>
#include <config.h>

#include <groundcontrol.h>
#include <sensori.h>
#include <attuatori.h>

#include <math/pid.h>

typedef struct __pilota_system_t {
	float roll_f;		//-1.0 <-> 1.0
	float pitch_f;		//-1.0 <-> 1.0
	float yaw_f;		//-1.0 <-> 1.0
	float throttle_f;   //0.0 <-> 1.0

	int16_t roll_out;   	//-10000 <-> 10000
	int16_t pitch_out;   	//-10000 <-> 10000
	int16_t yaw_out;   		//-10000 <-> 10000
	int16_t throttle_out;	//-10000 <-> 10000

	float heading;			//rad

//	int16_t rollspeed;
//	int16_t pitchspeed;
//	int16_t yawspeed;
//	int16_t throttlespeed;

//	float altitude_f;

//	int16_t speed;
//	int64_t time_dt;
} pilota_system_t;

typedef struct __pilota_location_t {
	int32_t longitude;
	int32_t latitude;
	int16_t altitude;

	int16_t heading;			//gradi sessagesimali * 100
} pilota_location_t;


typedef struct __pilota_t {
	pilota_location_t base;		//home
	pilota_system_t last;
	pilota_system_t current;
	groundcontrol_t groundcontrol;
	sensori_t       sensori;
	attuatori_t     attuatori;
} pilota_t;

pilota_t        pilota_data;
pthread_mutex_t pilota_mutex;
pthread_cond_t  pilota_cond;

PID_t altitudePID;

PID_t rollPID;
PID_t pitchPID;
PID_t yawPID;

void *pilota_loop(void *);

//MANGLER
void init_pilota(void);
void deinit_pilota(void);

void read_pilota(void);
void write_pilota(void);

int16_t get_pilota_throttle(void);
uint16_t get_pilota_throttle_0100(void);

//Flight Mangler
void storeBase(pilota_location_t *base);


#endif /* PILOTA_H_ */
