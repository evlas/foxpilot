/*
 * sensori.h
 *
 *  Created on: 27/05/2011
 *      Author: Vito Ammirata
 */

#ifndef SENSORI_H_
#define SENSORI_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>

typedef struct __imu_t {
	int16_t roll;			// (gradi sessagesimali * 100)
	int16_t pitch;			// (gradi sessagesimali * 100)
	int16_t yaw;			// (gradi sessagesimali * 100)
} imu_t;

typedef struct __gps_t {
	bool GPS_fix;
	int8_t	gps_sat;		// numero di satelliti
	int32_t longitude;		// (valore*10**7)
	int32_t latitude;		// (valore*10**7)
	int16_t altitude;		// centimetri
	int16_t speed;		    // metri al secondo * 100
} gps_t;

typedef struct __batteria_t {
	int32_t volt;			// milliVolt
	int32_t corrente;		// milliAmpere
	uint16_t stato;			// 0->0% 10000->100%
} batteria_t;

typedef struct __sensori_t {
	 imu_t		imu;
	 gps_t		gps;
	 batteria_t batteria;
} sensori_t;

sensori_t       sensori_data;
pthread_mutex_t sensori_mutex;
//pthread_cond_t  sensori_cond;

// MANGLES
void init_sensori();
void deinit_sensori();

sensori_t *read_sensori(sensori_t *dst);

void write_imu_sensori(imu_t *src);
imu_t *read_imu_sensori(imu_t *dst);

void write_gps_sensori(gps_t *src);
gps_t *read_gps_sensori(gps_t *dst);

void write_batteria_sensori(batteria_t *src);
batteria_t *read_batteria_sensori(batteria_t *dst);

#endif /* SENSORI_H_ */
