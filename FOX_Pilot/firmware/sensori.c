/*
 * sensori.c
 *
 *  Created on: 27/05/2011
 *      Author: Vito Ammirata
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>

#include <sensori.h>
#include <watchdog.h>

sensori_t sensori_data;
pthread_mutex_t sensori_mutex = PTHREAD_MUTEX_INITIALIZER;
//pthread_cond_t  sensori_cond  = PTHREAD_COND_INITIALIZER;

void init_sensori() {
	memset(&sensori_data, 0, sizeof(sensori_t));
	set_sensori_watchdog(true);
}
void deinit_sensori(sensori_t *a) {
	set_sensori_watchdog(false);
}

sensori_t *read_sensori(sensori_t *dst) {
	pthread_mutex_lock(&sensori_mutex);
	memcpy(dst, &sensori_data, sizeof(sensori_t));
	pthread_mutex_unlock(&sensori_mutex);
	return(dst);
}

void write_imu_sensori(imu_t *src) {
	pthread_mutex_lock(&sensori_mutex);
	memcpy(&(sensori_data.imu), src, sizeof(imu_t));
	pthread_mutex_unlock(&sensori_mutex);
	set_sensori_watchdog(true);
}
imu_t *read_imu_sensori(imu_t *dst)	{
	pthread_mutex_lock(&sensori_mutex);
	memcpy(dst, &(sensori_data.imu), sizeof(imu_t));
	pthread_mutex_unlock(&sensori_mutex);
	return(dst);
}

void write_gps_sensori(gps_t *src) {
	pthread_mutex_lock(&sensori_mutex);
	memcpy(&(sensori_data.gps), src, sizeof(gps_t));
	pthread_mutex_unlock(&sensori_mutex);
	set_sensori_watchdog(true);
}
gps_t *read_gps_sensori(gps_t *dst)	{
	pthread_mutex_lock(&sensori_mutex);
	memcpy(dst, &(sensori_data.gps), sizeof(gps_t));
	pthread_mutex_unlock(&sensori_mutex);
	return(dst);
}

void write_batteria_sensori(batteria_t *src) {
	pthread_mutex_lock(&sensori_mutex);
	memcpy(&(sensori_data.batteria), src, sizeof(batteria_t));
	pthread_mutex_unlock(&sensori_mutex);
	set_sensori_watchdog(true);
}
batteria_t *read_batteria_sensori(batteria_t *dst)	{
	pthread_mutex_lock(&sensori_mutex);
	memcpy(dst, &(sensori_data.batteria), sizeof(batteria_t));
	pthread_mutex_unlock(&sensori_mutex);
	return(dst);
}

