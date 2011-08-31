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
	int res_sens = 0, res_grct = 0, res_attu = 0, attu_appl = 0;
	long delta = 0, loop_d = 0;
	uint64_t t1, t2;
	//	struct timeval tv1, tv2;

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
			break;
		case MAV_MODE_MANUAL: {    ///< System is allowed to be active, under manual (RC) control
			int i;
			int roll_out, pitch_out, yaw_out;
			int throttle_out;
			int out_min = 500;	//5%

			if(pilota_data.groundcontrol.state.status > MAV_STATE_STANDBY) {
				throttle_out = pilota_data.current.throttle = (uint16_t)remap(pilota_data.groundcontrol.state.remote.thrust, 0.0, 1.0, 0, 10000);

				//float pid_calculate(&throttlePID, float sp, float val, float val_dot, float dt);

				//pilota_data.current.pitch = (int16_t)remap(pilota_data.groundcontrol.state.remote.pitch, -1.0, 1.0, -10000, 10000);
				//pilota_data.current.roll = (int16_t)remap(pilota_data.groundcontrol.state.remote.roll, -1.0, 1.0, -10000, 10000);
				//pilota_data.current.yawspeed = (int16_t)remap(pilota_data.groundcontrol.state.remote.yaw, -1.0, 1.0, -10000, 10000);

				throttle_out = (int)remap(pilota_data.groundcontrol.state.remote.thrust, 0.0, 1.0, -10000, 10000);

				yaw_out = (int)(remap(pilota_data.groundcontrol.state.remote.yaw, -1.0, 1.0, -10000, 10000));

				//X_FRAME
				//roll_out = (int)(remap(pilota_data.groundcontrol.state.remote.roll, -1.0, 1.0, -10000, 10000) * .707);
				//pitch_out = (int)(remap(pilota_data.groundcontrol.state.remote.pitch, -1.0, 1.0, -10000, 10000) * .707);
				roll_out = (int)remap(pilota_data.groundcontrol.state.remote.roll, -1.0, 1.0, -10000, 10000);
				pitch_out = (int)remap(pilota_data.groundcontrol.state.remote.pitch, -1.0, 1.0, -10000, 10000);

				//printf("throttle_out %d yaw_out %d roll_out %d pitch_out %d \n", throttle_out, yaw_out, roll_out, pitch_out);


				// left
				pilota_data.attuatori.value[1] = (int)throttle_out - roll_out - pitch_out; // FRONT
				pilota_data.attuatori.value[2] = (int)throttle_out - roll_out + pitch_out; // BACK

				// right
				pilota_data.attuatori.value[0] = (int)throttle_out + roll_out - pitch_out; // FRONT
				pilota_data.attuatori.value[3] = (int)throttle_out + roll_out + pitch_out; // BACK


				// Yaw input
				pilota_data.attuatori.value[0] -= yaw_out; // CW
				pilota_data.attuatori.value[1] += yaw_out; // CCW
				pilota_data.attuatori.value[2] -= yaw_out; // CW
				pilota_data.attuatori.value[3] += yaw_out; // CCW

				// We need to clip motor output at out_max. When cipping a motors
				// output we also need to compensate for the instability by
				// lowering the opposite motor by the same proportion. This
				// ensures that we retain control when one or more of the motors
				// is at its maximum output
				for (i=0; i<4; i++) {
					if (pilota_data.attuatori.value[i] > pilota_data.attuatori.max[i]) {
						// note that i^1 is the opposite motor
						switch(i){
						case 0:
							pilota_data.attuatori.value[2] -= (pilota_data.attuatori.value[0] - pilota_data.attuatori.max[0]);
							pilota_data.attuatori.value[0] = pilota_data.attuatori.max[0];
							break;
						case 1:
							pilota_data.attuatori.value[3] -= (pilota_data.attuatori.value[1] - pilota_data.attuatori.max[1]);
							pilota_data.attuatori.value[1] = pilota_data.attuatori.max[1];
							break;
						case 2:
							pilota_data.attuatori.value[0] -= (pilota_data.attuatori.value[2] - pilota_data.attuatori.max[2]);
							pilota_data.attuatori.value[2] = pilota_data.attuatori.max[2];
							break;
						case 3:
							pilota_data.attuatori.value[1] -= (pilota_data.attuatori.value[3] - pilota_data.attuatori.max[3]);
							pilota_data.attuatori.value[3] = pilota_data.attuatori.max[3];
							break;
						}
					}
				}

				// limit output so motors don't stop
				pilota_data.attuatori.value[0] = max(pilota_data.attuatori.value[0], pilota_data.attuatori.min[0] + out_min);
				pilota_data.attuatori.value[1] = max(pilota_data.attuatori.value[1], pilota_data.attuatori.min[1] + out_min);
				pilota_data.attuatori.value[2] = max(pilota_data.attuatori.value[2], pilota_data.attuatori.min[2] + out_min);
				pilota_data.attuatori.value[3] = max(pilota_data.attuatori.value[3], pilota_data.attuatori.min[3] + out_min);

				//printf("0 %d 1 %d 2 %d 3 %d\n",pilota_data.attuatori.value[0],pilota_data.attuatori.value[1],pilota_data.attuatori.value[2],pilota_data.attuatori.value[3]);

				memcpy(&pilota_data.last, &pilota_data.current, sizeof(pilota_system_t));

				write_pilota();
			}
		}
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

	pid_init(&altitudePID, 0.0, 0.0, 0.0, 0.0, 0, 0);
	pid_init(&rollPID, 0.0, 0.0, 0.0, 0.0, 0, 0);
	pid_init(&pitchPID, 0.0, 0.0, 0.0, 0.0, 0, 0);
	pid_init(&yawPID, 0.0, 0.0, 0.0, 0.0, 0, 0);

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

uint16_t get_pilota_throttle(void) {
	uint16_t res;
	pthread_mutex_lock(&pilota_mutex);

	res = pilota_data.last.throttle;

	pthread_mutex_unlock(&pilota_mutex);

	return(res);
}

//Flight Mangler
void storeBase(pilota_system_t *base) {
	pthread_mutex_lock(&pilota_mutex);

	memcpy(&(pilota_data.base),base,sizeof(pilota_system_t));

	pthread_mutex_unlock(&pilota_mutex);
}



