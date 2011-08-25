/*
 * attuatori.c
 *
 *  Created on: 27/05/2011
 *      Author: Vito Ammirata
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>

#include <attuatori.h>
#include <watchdog.h>

attuatori_t     attuatori_data;
pthread_mutex_t attuatori_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  attuatori_cond  = PTHREAD_COND_INITIALIZER;

void init_attuatori() {
	int i;
	for (i=0;i<NUMS_ATTUATORI;i++) {
		attuatori_data.id[i] = i;
		attuatori_data.min[i] = -10000;
		if ((i == 0) || (i == 1) || (i == 2) || (i == 3)) {
		attuatori_data.zero[i] = -10000;
		} else {
			attuatori_data.zero[i] = 0;
		}
		attuatori_data.max[i] = 10000;
		attuatori_data.value[i] = attuatori_data.zero[i];
	}
	set_attuatori_watchdog(true);
}
void deinit_attuatori() {
	set_attuatori_watchdog(false);
}

void write_attuatori(attuatori_t *a) {
	int i;
	pthread_mutex_lock(&attuatori_mutex);

	memcpy(&attuatori_data,a,sizeof(attuatori_t));

	pthread_mutex_unlock(&attuatori_mutex);
	pthread_cond_signal(&attuatori_cond);
}
attuatori_t *read_attuatori(attuatori_t *a) {
	int i;
	pthread_mutex_lock(&attuatori_mutex);
	pthread_cond_wait(&attuatori_cond, &attuatori_mutex);

	memcpy(a, &attuatori_data, sizeof(attuatori_t));

	pthread_mutex_unlock(&attuatori_mutex);
	set_attuatori_watchdog(true);
	return(a);
}

void set_attuatori(attuatori_t *src) {
	int i;
	bool change = false;

	pthread_mutex_lock(&attuatori_mutex);
	for(i=0;i<NUMS_ATTUATORI;i++) {
		attuatori_data.id[i] = src->id[i];
		attuatori_data.min[i] = src->min[i];
		attuatori_data.zero[i] = src->zero[i];
		attuatori_data.max[i] = src->max[i];

		if(attuatori_data.value[i] > attuatori_data.max[i]) {
			attuatori_data.value[i] = src->max[i];
			change = true;
		}
		if(attuatori_data.value[i] < attuatori_data.min[i]) {
			attuatori_data.value[i] = src->min[i];
			change = true;
		}
	}
	pthread_mutex_unlock(&attuatori_mutex);
	if (change == true) {
		pthread_cond_signal(&attuatori_cond);
	}
}

attuatori_t *get_attuatori(attuatori_t *a) {
	pthread_mutex_lock(&attuatori_mutex);
	memcpy(a, &attuatori_data, sizeof(attuatori_t));
	pthread_mutex_unlock(&attuatori_mutex);

	return(a);
}
/*
void write_attuatore(attuatori_t *a, int attuatore) {
	pthread_mutex_lock(&attuatori_mutex);

	memcpy(&attuatori_data[attuatore],a,sizeof(attuatori_t));

	pthread_mutex_unlock(&attuatori_mutex);
	pthread_cond_signal(&attuatori_cond);
}
attuatori_t *read_attuatore(attuatori_t *a, int attuatore) {
	pthread_mutex_lock(&attuatori_mutex);
	pthread_cond_wait(&attuatori_cond, &attuatori_mutex);

	memcpy(a, &attuatori_data[attuatore], sizeof(attuatori_t));

	pthread_mutex_unlock(&attuatori_mutex);
	set_attuatori_watchdog(true);
	return(a);
}

attuatori_t *get_attuatore(attuatori_t *a, int attuatore) {
	pthread_mutex_lock(&attuatori_mutex);

	memcpy(a, &attuatori_data[attuatore], sizeof(attuatori_t));

	pthread_mutex_unlock(&attuatori_mutex);
	return(a);
}
 */
