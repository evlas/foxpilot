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

pilota_t     pilota_data;
pthread_mutex_t pilota_mutex= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t pilota_cond= PTHREAD_COND_INITIALIZER;

void *pilota_loop(void *ptr) {
	int res_sens = 0, res_grct = 0, res_attu = 0, attu_appl = 0;
	long delta = 0, loop_d = 0;
	uint64_t t1, t2;
//	struct timeval tv1, tv2;

	PID_t altitudePID;
	PID_t rollPID;
	PID_t pitchPID;
	PID_t yawPID;

	pid_init(&altitudePID, 0.0, 0.0, 0.0, 0.0, 0, 0);
	pid_init(&rollPID, 0.0, 0.0, 0.0, 0.0, 0, 0);
	pid_init(&pitchPID, 0.0, 0.0, 0.0, 0.0, 0, 0);
	pid_init(&yawPID, 0.0, 0.0, 0.0, 0.0, 0, 0);

	init_pilota();

	while(1) {
//		gettimeofday(&tv1, NULL);
		t1 = microsSinceEpoch();

		read_pilota();

		pid_set_parameters(&altitudePID,
				pilota_data.groundcontrol.param[PARAM_PID_ALTI_KP],
				pilota_data.groundcontrol.param[PARAM_PID_ALTI_KI],
				pilota_data.groundcontrol.param[PARAM_PID_ALTI_KD],
				pilota_data.groundcontrol.param[PARAM_PID_ALTI_INTMAX]);

		pid_set_parameters(&rollPID,
				pilota_data.groundcontrol.param[PARAM_PID_ROLL_KP],
				pilota_data.groundcontrol.param[PARAM_PID_ROLL_KI],
				pilota_data.groundcontrol.param[PARAM_PID_ROLL_KD],
				pilota_data.groundcontrol.param[PARAM_PID_ROLL_INTMAX]);

		pid_set_parameters(&pitchPID,
				pilota_data.groundcontrol.param[PARAM_PID_PITCH_KP],
				pilota_data.groundcontrol.param[PARAM_PID_PITCH_KI],
				pilota_data.groundcontrol.param[PARAM_PID_PITCH_KD],
				pilota_data.groundcontrol.param[PARAM_PID_PITCH_INTMAX]);

		pid_set_parameters(&yawPID,
				pilota_data.groundcontrol.param[PARAM_PID_YAW_KP],
				pilota_data.groundcontrol.param[PARAM_PID_YAW_KI],
				pilota_data.groundcontrol.param[PARAM_PID_YAW_KD],
				pilota_data.groundcontrol.param[PARAM_PID_YAW_INTMAX]);

		//		if ((res_sens+res_grct)>0) {
		// altitude smoothing
		// ------------------
		//calc_altitude_error(&sensori);

		// How off is our heading?
		// -----------------------
		//calc_bearing_error(&sensori);

		update_system_statemachine();
		printf("Stato %d Mode %d Nav %d\n", get_sys_state_status(), get_sys_state_mode(), get_sys_state_nav_mode());

		switch(pilota_data.groundcontrol.state.nav_mode){
		case MAV_MODE_UNINIT:     ///< System is in undefined state
			break;
		case MAV_MODE_LOCKED:     ///< Motors are blocked, system is safe
			break;
		case MAV_MODE_MANUAL:     ///< System is allowed to be active, under manual (RC) control
			break;
		case MAV_MODE_GUIDED:     ///< System is allowed to be active, under autonomous control, manual setpoint
			break;
		case MAV_MODE_AUTO:       ///< System is allowed to be active, under autonomous control and navigation
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
			break;
		}

//		gettimeofday(&tv2, NULL);
//		delta= (tv2.tv_sec - tv1.tv_sec)*(10^6) + (tv2.tv_usec - tv1.tv_usec);
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

//Flight Mangler
void storeBase(pilota_system_t *base) {
	pthread_mutex_lock(&pilota_mutex);

	memcpy(&(pilota_data.base),base,sizeof(pilota_system_t));

	pthread_mutex_unlock(&pilota_mutex);
}



