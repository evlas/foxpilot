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

#include <groundcontrol.h>

attuatori_t     attuatori_data;
pthread_mutex_t attuatori_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  attuatori_cond  = PTHREAD_COND_INITIALIZER;

void init_attuatori() {
	int i;
	pthread_mutex_lock(&attuatori_mutex);

	attuatori_data.id[0] = get_param_value(PARAM_SERVO_0_ID);
	attuatori_data.min[0] = get_param_value(PARAM_SERVO_0_MIN);
	attuatori_data.zero[0] = get_param_value(PARAM_SERVO_0_ZERO);
	attuatori_data.max[0] = get_param_value(PARAM_SERVO_0_MAX);
	attuatori_data.value[0] = attuatori_data.zero[0];

	attuatori_data.id[1] = get_param_value(PARAM_SERVO_1_ID);
	attuatori_data.min[1] = get_param_value(PARAM_SERVO_1_MIN);
	attuatori_data.zero[1] = get_param_value(PARAM_SERVO_1_ZERO);
	attuatori_data.max[1] = get_param_value(PARAM_SERVO_1_MAX);
	attuatori_data.value[1] = attuatori_data.zero[1];

	attuatori_data.id[2] = get_param_value(PARAM_SERVO_2_ID);
	attuatori_data.min[2] = get_param_value(PARAM_SERVO_2_MIN);
	attuatori_data.zero[2] = get_param_value(PARAM_SERVO_2_ZERO);
	attuatori_data.max[2] = get_param_value(PARAM_SERVO_2_MAX);
	attuatori_data.value[2] = attuatori_data.zero[2];

	attuatori_data.id[3] = get_param_value(PARAM_SERVO_3_ID);
	attuatori_data.min[3] = get_param_value(PARAM_SERVO_3_MIN);
	attuatori_data.zero[3] = get_param_value(PARAM_SERVO_3_ZERO);
	attuatori_data.max[3] = get_param_value(PARAM_SERVO_3_MAX);
	attuatori_data.value[3] = attuatori_data.zero[3];

	attuatori_data.id[4] = get_param_value(PARAM_SERVO_4_ID);
	attuatori_data.min[4] = get_param_value(PARAM_SERVO_4_MIN);
	attuatori_data.zero[4] = get_param_value(PARAM_SERVO_4_ZERO);
	attuatori_data.max[4] = get_param_value(PARAM_SERVO_4_MAX);
	attuatori_data.value[4] = attuatori_data.zero[4];

	attuatori_data.id[5] = get_param_value(PARAM_SERVO_5_ID);
	attuatori_data.min[5] = get_param_value(PARAM_SERVO_5_MIN);
	attuatori_data.zero[5] = get_param_value(PARAM_SERVO_5_ZERO);
	attuatori_data.max[5] = get_param_value(PARAM_SERVO_5_MAX);
	attuatori_data.value[5] = attuatori_data.zero[5];

	attuatori_data.id[6] = get_param_value(PARAM_SERVO_6_ID);
	attuatori_data.min[6] = get_param_value(PARAM_SERVO_6_MIN);
	attuatori_data.zero[6] = get_param_value(PARAM_SERVO_6_ZERO);
	attuatori_data.max[6] = get_param_value(PARAM_SERVO_6_MAX);
	attuatori_data.value[6] = attuatori_data.zero[6];

	attuatori_data.id[7] = get_param_value(PARAM_SERVO_7_ID);
	attuatori_data.min[7] = get_param_value(PARAM_SERVO_7_MIN);
	attuatori_data.zero[7] = get_param_value(PARAM_SERVO_7_ZERO);
	attuatori_data.max[7] = get_param_value(PARAM_SERVO_7_MAX);
	attuatori_data.value[7] = attuatori_data.zero[7];

	pthread_mutex_unlock(&attuatori_mutex);
	set_attuatori_watchdog(true);
}
void deinit_attuatori() {
	set_attuatori_watchdog(false);
}

void write_attuatori(attuatori_t *a) {
	int i;
	pthread_mutex_lock(&attuatori_mutex);

	for(i=0;i<NUMS_ATTUATORI;i++) {
		attuatori_data.value[i]=a->value[i];
	}
	//memcpy(&attuatori_data,a,sizeof(attuatori_t));

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
