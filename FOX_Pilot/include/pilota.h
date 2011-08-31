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
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	uint16_t throttle;   //0 - 10000

	int16_t rollspeed;
	int16_t pitchspeed;
	int16_t yawspeed;
	int16_t throttlespeed;

	int32_t longitude;
	int32_t latitude;
	int16_t altitude;

	int16_t speed;
	int64_t time_dt;
} pilota_system_t;

typedef struct __pilota_t {
	pilota_system_t base;
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

uint16_t get_pilota_throttle(void);

//Flight Mangler
void storeBase(pilota_system_t *base);


#endif /* PILOTA_H_ */
