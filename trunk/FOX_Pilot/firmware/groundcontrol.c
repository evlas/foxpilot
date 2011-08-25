/*
 * groundcontrol.c
 *
 *  Created on: 03/ago/2011
 *      Author: Vito Ammirata
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>

#include <pilota.h>
#include <watchdog.h>
#include <pilota.h>
#include <groundcontrol.h>

groundcontrol_t groundcontrol_data;
pthread_mutex_t groundcontrol_mutex = PTHREAD_MUTEX_INITIALIZER;

//MANGLER
void init_groundcontrol() {
	memset(&groundcontrol_data, 0, sizeof(groundcontrol_t));
	param_defaults();
	set_groundcontrol_watchdog(true);
}

//legge da groundcontrol_data
int read_groundcontrol(groundcontrol_t *a) {
	pthread_mutex_lock(&groundcontrol_mutex);
	memcpy(a, &groundcontrol_data, sizeof(groundcontrol_t));
	pthread_mutex_unlock(&groundcontrol_mutex);
	//	return(a->mode);
}
//scrive su groundcontrol_data
int write_groundcontrol(groundcontrol_t *a) {
	pthread_mutex_lock(&groundcontrol_mutex);
	memcpy(&groundcontrol_data, a, sizeof(groundcontrol_t));
	pthread_mutex_unlock(&groundcontrol_mutex);
//	pthread_cond_signal(&pilota_cond);
	return(0);
}

/*
int write_mode_groundcontrol(uint8_t a) {
	pthread_mutex_lock(&groundcontrol_mutex);
	groundcontrol_data.mode = a;
	pthread_mutex_unlock(&groundcontrol_mutex);
	pthread_cond_signal(&pilota_cond);
}
 */

void set_failsafe_groundcontrol(bool failsafe) {
	pthread_mutex_lock(&groundcontrol_mutex);
	groundcontrol_data.state.failsafe = failsafe;
	pthread_mutex_unlock(&groundcontrol_mutex);
}
bool get_failsafe_groundcontrol() {
	bool res;
	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.state.failsafe;
	pthread_mutex_unlock(&groundcontrol_mutex);
	return(res);
}



