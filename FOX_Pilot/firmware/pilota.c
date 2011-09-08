/*
 * pilota.c
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#include <defines.h>
#include <config.h>

#include <groundcontrol.h>
#include <watchdog.h>
#include <sensori.h>
#include <attuatori.h>
#include <pilota.h>
#include <pilota/quad/quad.h>

#include <math/pid.h>
#include <math/range.h>

pilota_t     pilota_data;
pthread_mutex_t pilota_mutex= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t pilota_cond= PTHREAD_COND_INITIALIZER;

PID_t altitudePID;
PID_t rollPID;
PID_t pitchPID;
PID_t yawPID;

void *pilota_loop(void *ptr) {
	int i, res_sens = 0, res_grct = 0, res_attu = 0, attu_appl = 0;
	long delta = 0, loop_d = 0;
	uint64_t t1, t2;

	init_pilota();

	get_attuatori(&pilota_data.attuatori);

	while(1) {
		//		gettimeofday(&tv1, NULL);
		t1 = microsSinceEpoch();

		read_pilota();

		//		if ((res_sens+res_grct)>0) {
		// altitude smoothing
		// ------------------
		//calc_altitude_error(&sensori);

		// How off is our heading?
		// -----------------------
		//calc_bearing_error(&sensori);

		update_system_statemachine();
		//printf("Stato %d Mode %d Nav %d\n", get_sys_state_status(), get_sys_state_mode(), get_sys_state_nav_mode());

		switch(pilota_data.groundcontrol.state.mav_mode){
		case MAV_MODE_UNINIT:     ///< System is in undefined state
			break;
		case MAV_MODE_LOCKED:     ///< Motors are blocked, system is safe
			for(i=0;i<NUMS_ATTUATORI;i++) {
				if ((pilota_data.attuatori.id[i] = 0)
						|| (pilota_data.attuatori.id[i] = 1)
						|| (pilota_data.attuatori.id[i] = 2)
						|| (pilota_data.attuatori.id[i] = 3)) {
					pilota_data.attuatori.value[i] = pilota_data.attuatori.min[i];
				}
			}
			write_pilota();
			break;
		case MAV_MODE_MANUAL:     ///< System is allowed to be active, under manual (RC) control
			if(processFlightControlQuadXManual(&pilota_data)==0){
				write_pilota();
			}
			break;
		case MAV_MODE_GUIDED:     ///< System is allowed to be active, under autonomous control, manual setpoint
			if(processFlightControlQuadX(&pilota_data)==0){
				write_pilota();
			}
			break;
		case MAV_MODE_AUTO:       ///< System is allowed to be active, under autonomous control and navigation
			if(processFlightControlQuadXAuto(&pilota_data)==0){
				write_pilota();
			}
			break;
		case MAV_MODE_TEST1:      ///< Generic test mode, for custom use
			break;
		case MAV_MODE_TEST2:      ///< Generic test mode, for custom use
			break;
		case MAV_MODE_TEST3:      ///< Generic test mode, for custom use
			break;
		case MAV_MODE_READY:      ///< System is ready, motors are unblocked, but controllers are inactive
			break;
		case MAV_MODE_RC_TRAINING: ///< System is blocked, only RC valued are read and reported back
			processFlightControlQuadXManual(&pilota_data);
			break;
		}

		t2 = microsSinceEpoch();
		delta = t2 - t1;

		//		printf("delta %ld\n",delta);

		loop_d = PERIODO_PILOTA - delta;

		if (loop_d > 0) {
			//attendo (1/50 sec - tempo impiegato fino a qui) microsec
			usleep(loop_d);
		}
		set_pilota_watchdog(true);
	}
	set_pilota_watchdog(false);
}

// INIT
void init_pilota() {
	memset(&pilota_data, 0, sizeof(pilota_t));

	pid_init(&altitudePID,
			get_param_value(PARAM_PID_ALTI_KP),
			get_param_value(PARAM_PID_ALTI_KI),
			get_param_value(PARAM_PID_ALTI_KD),
			get_param_value(PARAM_PID_ALTI_INTMAX),
			PID_MODE_DERIVATIV_CALC,
			0);

	pid_init(&rollPID,
			get_param_value(PARAM_PID_ROLL_KP),
			get_param_value(PARAM_PID_ROLL_KI),
			get_param_value(PARAM_PID_ROLL_KD),
			get_param_value(PARAM_PID_ROLL_INTMAX),
			PID_MODE_DERIVATIV_CALC,
			0);
	pid_init(&pitchPID,
			get_param_value(PARAM_PID_PITCH_KP),
			get_param_value(PARAM_PID_PITCH_KI),
			get_param_value(PARAM_PID_PITCH_KD),
			get_param_value(PARAM_PID_PITCH_INTMAX),
			PID_MODE_DERIVATIV_CALC,
			0);
	pid_init(&yawPID,
			get_param_value(PARAM_PID_YAW_KP),
			get_param_value(PARAM_PID_YAW_KI),
			get_param_value(PARAM_PID_YAW_KD),
			get_param_value(PARAM_PID_YAW_INTMAX),
			PID_MODE_DERIVATIV_CALC,
			0);

	set_pilota_watchdog(true);
}

void read_pilota() {
	pthread_mutex_lock(&pilota_mutex);
	//	pthread_cond_wait(&pilota_cond, &pilota_mutex);

	read_sensori(&pilota_data.sensori);
	read_groundcontrol(&pilota_data.groundcontrol);

	pthread_mutex_unlock(&pilota_mutex);
}

void write_pilota() {
	write_attuatori(&(pilota_data.attuatori));
	set_pilota_watchdog(true);
}

int16_t get_pilota_throttle(void) {
	int16_t res;
	pthread_mutex_lock(&pilota_mutex);

	res = pilota_data.last.throttle_out;

	pthread_mutex_unlock(&pilota_mutex);

	return(res);
}

uint16_t get_pilota_throttle_0100(void) {
	int16_t res;
	pthread_mutex_lock(&pilota_mutex);

	res = remap(pilota_data.last.throttle_out,-10000,10000,0,10000);

	pthread_mutex_unlock(&pilota_mutex);

	return(res);
}

//Flight Mangler
void storeBase(pilota_location_t *base) {
	pthread_mutex_lock(&pilota_mutex);

	memcpy(&(pilota_data.base),base,sizeof(pilota_location_t));

	pthread_mutex_unlock(&pilota_mutex);
}



