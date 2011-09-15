/*
 * quad.c
 *
 *  Created on: 26/apr/2011
 *      Author: vito
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <math.h>

#include <defines.h>
#include <config.h>

#include <groundcontrol.h>
#include <watchdog.h>
#include <sensori.h>
#include <attuatori.h>
#include <pilota.h>

#include <math/pid.h>
#include <math/range.h>

#include "quad.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// X MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int processFlightControlQuadXGuided(pilota_t *pilota_quad) {
	switch(pilota_quad->groundcontrol.state.fly) {
	case FLY_IDLE:
		printf("FLY_IDLE\n");
		break;
	case FLY_STARTING:
		printf("FLY_STARTING\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_IDLE) {
			if(pilota_quad->groundcontrol.state.remote.thrust > 0) {
				set_sys_state_fly(FLY_TAKE_OFF);
			}
		}
		break;
	case FLY_TAKE_OFF:
		printf("FLY_TAKE_OFF\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_STARTING) {
			if ((take_off(pilota_quad, 100.0, MAV_FRAME_GLOBAL_RELATIVE_ALT) == 0) && (pilota_quad->groundcontrol.state.remote.thrust > 0)) {
				set_sys_state_fly(FLY_FLYING);
			}
		}
		break;
	case FLY_FLYING:
		printf("FLY_FLYING\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_TAKE_OFF) {
			if (pilota_quad->groundcontrol.state.remote.thrust < 0.001) {
				set_sys_state_fly(FLY_LANDING);
			} else {
				guided_fly_handler(pilota_quad);
			}
		}
		break;
	case FLY_LANDING:
		printf("FLY_LANDING\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_FLYING){
			if (landing(pilota_quad) == 0) {
				set_sys_state_fly(FLY_END);
			}
		}
		break;
	case FLY_END:
		printf("FLY_END\n");
		if ((pilota_quad->groundcontrol.state.prevfly == FLY_LANDING) || (pilota_quad->groundcontrol.state.prevfly == FLY_FLYING)) {
			set_sys_state_nav_mode(MAV_NAV_GROUNDED);
			set_sys_state_fly(FLY_IDLE);
		}
		break;
	}
}

int processFlightControlQuadXManual(pilota_t *pilota_quad) {
	int i;
	int out_min = 500;	//5%

	if(pilota_quad->groundcontrol.state.status > MAV_STATE_STANDBY) {
		pilota_quad->current.throttle_out = (int16_t)(remap(pilota_quad->groundcontrol.state.remote.thrust, 0.0, 1.0, -10000, 10000));

		//		pilota_quad->current.yaw = (int16_t)(remap(pilota_quad->groundcontrol.state.remote.yaw, -1.0, 1.0, -10000, 10000));
		pilota_quad->current.yaw_out = angle_to_10000(pilota_quad->groundcontrol.state.remote.yaw);

		//X_FRAME
		//pilota_quad->current.roll = (int16_t)remap(pilota_quad->groundcontrol.state.remote.roll, -1.0, 1.0, -10000, 10000);
		//pilota_quad->current.pitch = (int16_t)remap(pilota_quad->groundcontrol.state.remote.pitch, -1.0, 1.0, -10000, 10000);
		pilota_quad->current.roll_out = angle_to_10000(pilota_quad->groundcontrol.state.remote.roll);
		pilota_quad->current.pitch_out = angle_to_10000(pilota_quad->groundcontrol.state.remote.pitch);

		// left
		pilota_quad->attuatori.value[1] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
		pilota_quad->attuatori.value[2] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

		// right
		pilota_quad->attuatori.value[0] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
		pilota_quad->attuatori.value[3] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

		// Yaw input
		pilota_quad->attuatori.value[0] += pilota_quad->current.yaw_out; // CCW
		pilota_quad->attuatori.value[1] -= pilota_quad->current.yaw_out; // CW
		pilota_quad->attuatori.value[2] += pilota_quad->current.yaw_out; // CCW
		pilota_quad->attuatori.value[3] -= pilota_quad->current.yaw_out; // CW

		// We need to clip motor output at out_max. When cipping a motors
		// output we also need to compensate for the instability by
		// lowering the opposite motor by the same proportion. This
		// ensures that we retain control when one or more of the motors
		// is at its maximum output
		for (i=0; i<4; i++) {
			if (pilota_quad->attuatori.value[i] > pilota_quad->attuatori.max[i]) {
				// note that i^1 is the opposite motor
				switch(i){
				case 0:
					pilota_quad->attuatori.value[2] -= (pilota_quad->attuatori.value[0] - pilota_quad->attuatori.max[0]);
					pilota_quad->attuatori.value[0] = pilota_quad->attuatori.max[0];
					break;
				case 1:
					pilota_quad->attuatori.value[3] -= (pilota_quad->attuatori.value[1] - pilota_quad->attuatori.max[1]);
					pilota_quad->attuatori.value[1] = pilota_quad->attuatori.max[1];
					break;
				case 2:
					pilota_quad->attuatori.value[0] -= (pilota_quad->attuatori.value[2] - pilota_quad->attuatori.max[2]);
					pilota_quad->attuatori.value[2] = pilota_quad->attuatori.max[2];
					break;
				case 3:
					pilota_quad->attuatori.value[1] -= (pilota_quad->attuatori.value[3] - pilota_quad->attuatori.max[3]);
					pilota_quad->attuatori.value[3] = pilota_quad->attuatori.max[3];
					break;
				}
			}
		}

		// limit output so motors don't stop
		pilota_quad->attuatori.value[0] = max(pilota_quad->attuatori.value[0], pilota_quad->attuatori.min[0] + out_min);
		pilota_quad->attuatori.value[1] = max(pilota_quad->attuatori.value[1], pilota_quad->attuatori.min[1] + out_min);
		pilota_quad->attuatori.value[2] = max(pilota_quad->attuatori.value[2], pilota_quad->attuatori.min[2] + out_min);
		pilota_quad->attuatori.value[3] = max(pilota_quad->attuatori.value[3], pilota_quad->attuatori.min[3] + out_min);

		//printf("0 %d 1 %d 2 %d 3 %d\n",pilota_quad->attuatori.value[0],pilota_quad->attuatori.value[1],pilota_quad->attuatori.value[2],pilota_quad->attuatori.value[3]);

		pilota_quad->last = pilota_quad->current;
	} else {
		return(1);
	}
	return(0);
}

int processFlightControlQuadXAuto(pilota_t *pilota_quad) {
	mavlink_waypoint_t waypoint;

	switch(pilota_quad->groundcontrol.state.fly) {
	case FLY_IDLE:
		printf("FLY_IDLE\n");
		break;
	case FLY_STARTING:
		printf("FLY_STARTING\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_IDLE) {
			get_waypoint(get_waypoint_current_active_wp_id(), &waypoint);

			printf("waypoint_active.autocontinue %d\n", waypoint.autocontinue);
			printf("waypoint_active.command %d\n", waypoint.command);
			printf("waypoint_active.current %d\n", waypoint.current);
			printf("waypoint_active.frame %d\n", waypoint.frame);
			printf("waypoint_active.param1 %f\n", waypoint.param1);
			printf("waypoint_active.param2 %f\n", waypoint.param2);
			printf("waypoint_active.param3 %f\n", waypoint.param3);
			printf("waypoint_active.param4 %f\n", waypoint.param4);
			printf("waypoint_active.seq %d\n", waypoint.seq);
			printf("waypoint_active.x %f\n", waypoint.x);
			printf("waypoint_active.y %f\n", waypoint.y);
			printf("waypoint_active.z %f\n", waypoint.z);

			if ((waypoint.current == 1) || (waypoint.command == MAV_CMD_NAV_TAKEOFF)) {
				set_sys_state_fly(FLY_TAKE_OFF);
			}
		}
		break;
	case FLY_TAKE_OFF:
		printf("FLY_TAKE_OFF\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_STARTING) {
			get_waypoint(get_waypoint_current_active_wp_id(), &waypoint);

			printf("waypoint_active.autocontinue %d\n", waypoint.autocontinue);
			printf("waypoint_active.command %d\n", waypoint.command);
			printf("waypoint_active.current %d\n", waypoint.current);
			printf("waypoint_active.frame %d\n", waypoint.frame);
			printf("waypoint_active.param1 %f\n", waypoint.param1);
			printf("waypoint_active.param2 %f\n", waypoint.param2);
			printf("waypoint_active.param3 %f\n", waypoint.param3);
			printf("waypoint_active.param4 %f\n", waypoint.param4);
			printf("waypoint_active.seq %d\n", waypoint.seq);
			printf("waypoint_active.x %f\n", waypoint.x);
			printf("waypoint_active.y %f\n", waypoint.y);
			printf("waypoint_active.z %f\n", waypoint.z);

			if (take_off(pilota_quad, waypoint.z*100, waypoint.frame) == 0) {
				set_waypoint_current_active_wp_id(get_waypoint_current_active_wp_id()+1);
				set_sys_state_fly(FLY_FLYING);
			}
		}
		break;
	case FLY_FLYING:
		printf("FLY_FLYING\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_TAKE_OFF){
			auto_fly_handler(pilota_quad);
		}
		break;
	case FLY_LANDING:
		printf("FLY_LANDING\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_FLYING){
			if (landing(pilota_quad) == 0) {
				set_sys_state_fly(FLY_END);
			}
		}
		break;
	case FLY_END:
		printf("FLY_END\n");
		if (pilota_quad->groundcontrol.state.prevfly == FLY_LANDING) {
			set_sys_state_nav_mode(MAV_NAV_GROUNDED);
		}
		break;
	}
}

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int processFlightControlQuadPlusGuided(pilota_t *pilota_quad) {
}

int processFlightControlQuadPlusMaual(pilota_t *pilota_quad) {
}

int processFlightControlQuadPlusAuto(pilota_t *pilota_quad) {
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// guided fly mode //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int guided_fly_handler(pilota_t *pilota_quad) {
	int i;
	int out_min = 500;	//5%

	if (pilota_quad->groundcontrol.state.status > MAV_STATE_STANDBY) {
		processAttitude(pilota_quad, 0.0, 0.0);
		processHeading(pilota_quad, 0.0);
		processThrottle(pilota_quad, 0.0);

		//////////////
		pilota_quad->current.throttle_out = (uint16_t)(remap(pilota_quad->current.throttle_f, 0.0, 1.0, -10000, 10000));

		//pilota_quad->current.yaw = (uint16_t)(remap(pilota_quad->current.yaw_f, -1.0, 1.0, -10000, 10000));
		pilota_quad->current.yaw_out = angle_to_10000(pilota_quad->current.yaw_f);

		//X_FRAME
		//roll_out = (int)(remap(pilota_quad->groundcontrol.state.remote.roll, -1.0, 1.0, -10000, 10000) * .707);
		//pitch_out = (int)(remap(pilota_quad->groundcontrol.state.remote.pitch, -1.0, 1.0, -10000, 10000) * .707);
		//pilota_quad->current.roll = (uint16_t)remap(pilota_quad->current.roll_f, -1.0, 1.0, -10000, 10000);
		//pilota_quad->current.pitch = (uint16_t)remap(pilota_quad->current.pitch_f, -1.0, 1.0, -10000, 10000);
		pilota_quad->current.roll_out = angle_to_10000(pilota_quad->current.roll_f);
		pilota_quad->current.pitch_out = angle_to_10000(pilota_quad->current.pitch_f);

		//printf("throttle_out %d yaw_out %d roll_out %d pitch_out %d \n", throttle_out, yaw_out, roll_out, pitch_out);

		// left
		pilota_quad->attuatori.value[1] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
		pilota_quad->attuatori.value[2] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

		// right
		pilota_quad->attuatori.value[0] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
		pilota_quad->attuatori.value[3] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

		// Yaw input
		pilota_quad->attuatori.value[0] += pilota_quad->current.yaw_out; // CCW
		pilota_quad->attuatori.value[1] -= pilota_quad->current.yaw_out; // CW
		pilota_quad->attuatori.value[2] += pilota_quad->current.yaw_out; // CCW
		pilota_quad->attuatori.value[3] -= pilota_quad->current.yaw_out; // CW

		// We need to clip motor output at out_max. When cipping a motors
		// output we also need to compensate for the instability by
		// lowering the opposite motor by the same proportion. This
		// ensures that we retain control when one or more of the motors
		// is at its maximum output
		for (i=0; i<4; i++) {
			if (pilota_quad->attuatori.value[i] > pilota_quad->attuatori.max[i]) {
				// note that i^1 is the opposite motor
				switch(i){
				case 0:
					pilota_quad->attuatori.value[2] -= (pilota_quad->attuatori.value[0] - pilota_quad->attuatori.max[0]);
					pilota_quad->attuatori.value[0] = pilota_quad->attuatori.max[0];
					break;
				case 1:
					pilota_quad->attuatori.value[3] -= (pilota_quad->attuatori.value[1] - pilota_quad->attuatori.max[1]);
					pilota_quad->attuatori.value[1] = pilota_quad->attuatori.max[1];
					break;
				case 2:
					pilota_quad->attuatori.value[0] -= (pilota_quad->attuatori.value[2] - pilota_quad->attuatori.max[2]);
					pilota_quad->attuatori.value[2] = pilota_quad->attuatori.max[2];
					break;
				case 3:
					pilota_quad->attuatori.value[1] -= (pilota_quad->attuatori.value[3] - pilota_quad->attuatori.max[3]);
					pilota_quad->attuatori.value[3] = pilota_quad->attuatori.max[3];
					break;
				}
			}
		}

		// limit output so motors don't stop
		pilota_quad->attuatori.value[0] = max(pilota_quad->attuatori.value[0], pilota_quad->attuatori.min[0] + out_min);
		pilota_quad->attuatori.value[1] = max(pilota_quad->attuatori.value[1], pilota_quad->attuatori.min[1] + out_min);
		pilota_quad->attuatori.value[2] = max(pilota_quad->attuatori.value[2], pilota_quad->attuatori.min[2] + out_min);
		pilota_quad->attuatori.value[3] = max(pilota_quad->attuatori.value[3], pilota_quad->attuatori.min[3] + out_min);

		//printf("0 %d 1 %d 2 %d 3 %d\n",pilota_quad->attuatori.value[0],pilota_quad->attuatori.value[1],pilota_quad->attuatori.value[2],pilota_quad->attuatori.value[3]);

		pilota_quad->last = pilota_quad->current;

	} else {
		return (1);
	}

	return(0);
}
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// auto fly mode   //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int	auto_fly_handler(pilota_t *pilota_quad) {
	int i;
	int out_min = 500;	//5%
	mavlink_waypoint_t waypoint_active;

	printf("waypoint_fly_handler\n");

	get_waypoint(get_waypoint_current_active_wp_id(),&waypoint_active);

	printf("waypoint_active.current %d\n", waypoint_active.current);

	send_mav_waypoint_current(get_waypoint_current_active_wp_id());

	if (pilota_quad->groundcontrol.state.status > MAV_STATE_STANDBY) {
		//processAttitudeAuto(pilota_quad, roll, pitch);

		//processHeadingAuto(pilota_quad, heading);

		//processAltitude(pilota_quad);
	}

	return(0);
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAttitude //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAttitude(pilota_t *pilota_quad, float roll, float pitch) {
	if (pilota_quad->groundcontrol.state.mav_mode == MAV_MODE_GUIDED) {
		//pilota_quad->groundcontrol.state.remote.roll	-0.2 <-> 0.2 rad
		pilota_quad->current.roll_f = pid_calculate(&rollPID,
				pilota_quad->groundcontrol.state.remote.roll,
				pilota_quad->sensori.imu.roll*D2R/100.0,
				0.0);

		//pilota_quad->groundcontrol.state.remote.pitch	-0.2 <-> 0.2 rad
		pilota_quad->current.pitch_f = pid_calculate(&pitchPID,
				pilota_quad->groundcontrol.state.remote.pitch,
				pilota_quad->sensori.imu.pitch*D2R/100.0,
				0.0);
	} else if (pilota_quad->groundcontrol.state.mav_mode == MAV_MODE_AUTO) {
		//pilota_quad->groundcontrol.state.remote.roll	-0.2 <-> 0.2 rad
		pilota_quad->current.roll_f = pid_calculate(&rollPID,
				roll,
				pilota_quad->sensori.imu.roll*D2R/100.0,
				0.0);

		//pilota_quad->groundcontrol.state.remote.pitch	-0.2 <-> 0.2 rad
		pilota_quad->current.pitch_f = pid_calculate(&pitchPID,
				pitch,
				pilota_quad->sensori.imu.pitch*D2R/100.0,
				0.0);
	}

	pilota_quad->current.roll_f = clamp(pilota_quad->current.roll_f,-0.4,0.4);
	pilota_quad->current.pitch_f = clamp(pilota_quad->current.pitch_f,-0.4,0.4);

	printf("pilota_quad->current.roll_f %f\n", pilota_quad->current.roll_f);
	printf("pilota_quad->current.pitch_f %f\n", pilota_quad->current.pitch_f);
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeading ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading(pilota_t *pilota_quad, float heading) {
	float yaw_incr;

	if (pilota_quad->groundcontrol.state.mav_mode == MAV_MODE_GUIDED) {
		//pilota_quad->groundcontrol.state.remote.yaw	-0.5 <-> 0.5 rad
		yaw_incr = pilota_quad->groundcontrol.state.remote.yaw + pilota_quad->last.heading;
		if (yaw_incr > PI) {
			yaw_incr -= 2*PI;
		}
		if (yaw_incr < -PI)	{
			yaw_incr += 2*PI;
		}

		pilota_quad->current.yaw_f = pid_calculate(&yawPID,
				yaw_incr,
				pilota_quad->sensori.imu.yaw*D2R/100.0,
				0.0);
	} else if (pilota_quad->groundcontrol.state.mav_mode == MAV_MODE_AUTO) {
		pilota_quad->current.yaw_f = pid_calculate(&yawPID,
				heading,
				pilota_quad->sensori.imu.yaw*D2R/100.0,
				0.0);
	}

	pilota_quad->current.yaw_f = clamp(pilota_quad->current.yaw_f,-0.5,0.5);
	pilota_quad->current.heading = heading;

	printf("pilota_quad->current.yaw_f %f\n", pilota_quad->current.yaw_f);
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processThrottle //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processThrottle(pilota_t *pilota_quad, float throttle) {
	if (pilota_quad->groundcontrol.state.mav_mode == MAV_MODE_GUIDED) {
		pilota_quad->current.throttle_f = pilota_quad->groundcontrol.state.remote.thrust *
				(2.0 - (cos(pilota_quad->current.roll_f)*cos(pilota_quad->current.pitch_f)));
	} else if (pilota_quad->groundcontrol.state.mav_mode == MAV_MODE_AUTO) {
		pilota_quad->current.throttle_f = throttle *
				(2.0 - (cos(pilota_quad->current.roll_f)*cos(pilota_quad->current.pitch_f)));
	}

	pilota_quad->current.throttle_f = clamp(pilota_quad->current.throttle_f,0.0,1.0);

	printf("pilota_quad->current.throttle_f %f\n", pilota_quad->current.throttle_f);
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAltitude //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAltitude(pilota_t *pilota_quad, float altitude) {
}

int take_off(pilota_t *pilota_quad, float alti, enum MAV_FRAME frame) {
	uint16_t altezza0, altezzaEnd;
	int end = 0;
	int i;
	int out_min = 0;	//5%

	altezza0 = pilota_quad->sensori.gps.altitude;

	switch(frame) {
	case MAV_FRAME_GLOBAL:
		altezzaEnd = (uint16_t)alti;
		break;
	case MAV_FRAME_LOCAL:
		send_mav_statustext(0,"MAV_FRAME_LOCAL not supported",29);
		return(1);
		break;
	case MAV_FRAME_MISSION:
		send_mav_statustext(0,"MAV_FRAME_MISSION not supported",31);
		return(1);
		break;
	case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		altezzaEnd = altezza0 + (uint16_t)alti;
		break;
	case MAV_FRAME_LOCAL_ENU:
		send_mav_statustext(0,"MAV_FRAME_LOCAL_ENU not supported",33);
		return(1);
		break;
	};

	do {
		read_pilota();

		if (pilota_quad->groundcontrol.state.status > MAV_STATE_STANDBY) {
			processAttitude(pilota_quad, 0.0, 0.0);
			processHeading(pilota_quad, 0.0);

			if(pilota_quad->sensori.gps.altitude < (altezzaEnd - 5)) {
				pilota_quad->current.throttle_f = pilota_quad->last.throttle_f + 0.0001;
			}
			if (pilota_quad->sensori.gps.altitude > (altezzaEnd + 5)) {
				pilota_quad->current.throttle_f = pilota_quad->last.throttle_f - 0.0001;
			}

			pilota_quad->current.throttle_f = pilota_quad->last.throttle_f + 0.0001;
			pilota_quad->current.throttle_f = clamp(pilota_quad->current.throttle_f,0.0,1.0);

			printf ("pilota_quad->current.throttle_f %f\n",pilota_quad->current.throttle_f);

			//////////////
			pilota_quad->current.throttle_out = (uint16_t)(remap(pilota_quad->current.throttle_f, 0.0, 1.0, -10000, 10000));

			//pilota_quad->current.yaw = (uint16_t)(remap(pilota_quad->current.yaw_f, -1.0, 1.0, -10000, 10000));
			pilota_quad->current.yaw_out = angle_to_10000(pilota_quad->current.yaw_f);

			//X_FRAME
			//roll_out = (int)(remap(pilota_quad->groundcontrol.state.remote.roll, -1.0, 1.0, -10000, 10000) * .707);
			//pitch_out = (int)(remap(pilota_quad->groundcontrol.state.remote.pitch, -1.0, 1.0, -10000, 10000) * .707);
			//pilota_quad->current.roll = (uint16_t)remap(pilota_quad->current.roll_f, -1.0, 1.0, -10000, 10000);
			//pilota_quad->current.pitch = (uint16_t)remap(pilota_quad->current.pitch_f, -1.0, 1.0, -10000, 10000);
			pilota_quad->current.roll_out = angle_to_10000(pilota_quad->current.roll_f);
			pilota_quad->current.pitch_out = angle_to_10000(pilota_quad->current.pitch_f);

			//printf("throttle_out %d yaw_out %d roll_out %d pitch_out %d \n", throttle_out, yaw_out, roll_out, pitch_out);

			// left
			pilota_quad->attuatori.value[1] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
			pilota_quad->attuatori.value[2] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

			// right
			pilota_quad->attuatori.value[0] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
			pilota_quad->attuatori.value[3] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

			// Yaw input
			pilota_quad->attuatori.value[0] += pilota_quad->current.yaw_out; // CCW
			pilota_quad->attuatori.value[1] -= pilota_quad->current.yaw_out; // CW
			pilota_quad->attuatori.value[2] += pilota_quad->current.yaw_out; // CCW
			pilota_quad->attuatori.value[3] -= pilota_quad->current.yaw_out; // CW

			// We need to clip motor output at out_max. When cipping a motors
			// output we also need to compensate for the instability by
			// lowering the opposite motor by the same proportion. This
			// ensures that we retain control when one or more of the motors
			// is at its maximum output
			for (i=0; i<4; i++) {
				if (pilota_quad->attuatori.value[i] > pilota_quad->attuatori.max[i]) {
					// note that i^1 is the opposite motor
					switch(i){
					case 0:
						pilota_quad->attuatori.value[2] -= (pilota_quad->attuatori.value[0] - pilota_quad->attuatori.max[0]);
						pilota_quad->attuatori.value[0] = pilota_quad->attuatori.max[0];
						break;
					case 1:
						pilota_quad->attuatori.value[3] -= (pilota_quad->attuatori.value[1] - pilota_quad->attuatori.max[1]);
						pilota_quad->attuatori.value[1] = pilota_quad->attuatori.max[1];
						break;
					case 2:
						pilota_quad->attuatori.value[0] -= (pilota_quad->attuatori.value[2] - pilota_quad->attuatori.max[2]);
						pilota_quad->attuatori.value[2] = pilota_quad->attuatori.max[2];
						break;
					case 3:
						pilota_quad->attuatori.value[1] -= (pilota_quad->attuatori.value[3] - pilota_quad->attuatori.max[3]);
						pilota_quad->attuatori.value[3] = pilota_quad->attuatori.max[3];
						break;
					}
				}
			}

			// limit output so motors don't stop
			pilota_quad->attuatori.value[0] = max(pilota_quad->attuatori.value[0], pilota_quad->attuatori.min[0] + out_min);
			pilota_quad->attuatori.value[1] = max(pilota_quad->attuatori.value[1], pilota_quad->attuatori.min[1] + out_min);
			pilota_quad->attuatori.value[2] = max(pilota_quad->attuatori.value[2], pilota_quad->attuatori.min[2] + out_min);
			pilota_quad->attuatori.value[3] = max(pilota_quad->attuatori.value[3], pilota_quad->attuatori.min[3] + out_min);

			//printf("0 %d 1 %d 2 %d 3 %d\n",pilota_quad->attuatori.value[0],pilota_quad->attuatori.value[1],pilota_quad->attuatori.value[2],pilota_quad->attuatori.value[3]);

			pilota_quad->last = pilota_quad->current;

		}
		write_pilota();
		usleep(20000);

		if((pilota_quad->sensori.gps.altitude > (altezzaEnd - 5)) && (pilota_quad->sensori.gps.altitude < (altezzaEnd + 5))) {
			end = 1;
		}

	} while (end < 1);

	return(0);
}
int landing(pilota_t *pilota_quad) {
	uint16_t altezza100, altezzaEnd;
	int end = 0;
	int i;
	int out_min = 0;	//5%

	altezza100 = pilota_quad->sensori.gps.altitude;
	altezzaEnd = altezza100 - 100;

	do {
		read_pilota();

		if (pilota_quad->groundcontrol.state.status > MAV_STATE_STANDBY) {
			processAttitude(pilota_quad,0.0,0.0);
			processHeading(pilota_quad,0.0);

			pilota_quad->current.throttle_f = pilota_quad->last.throttle_f - 0.0001;
			pilota_quad->current.throttle_f = clamp(pilota_quad->current.throttle_f,0.0,1.0);

			printf ("pilota_quad->current.throttle_f %f\n",pilota_quad->current.throttle_f);

			//////////////
			pilota_quad->current.throttle_out = (uint16_t)(remap(pilota_quad->current.throttle_f, 0.0, 1.0, -10000, 10000));

			//pilota_quad->current.yaw = (uint16_t)(remap(pilota_quad->current.yaw_f, -1.0, 1.0, -10000, 10000));
			pilota_quad->current.yaw_out = angle_to_10000(pilota_quad->current.yaw_f);

			//X_FRAME
			//roll_out = (int)(remap(pilota_quad->groundcontrol.state.remote.roll, -1.0, 1.0, -10000, 10000) * .707);
			//pitch_out = (int)(remap(pilota_quad->groundcontrol.state.remote.pitch, -1.0, 1.0, -10000, 10000) * .707);
			//pilota_quad->current.roll = (uint16_t)remap(pilota_quad->current.roll_f, -1.0, 1.0, -10000, 10000);
			//pilota_quad->current.pitch = (uint16_t)remap(pilota_quad->current.pitch_f, -1.0, 1.0, -10000, 10000);
			pilota_quad->current.roll_out = angle_to_10000(pilota_quad->current.roll_f);
			pilota_quad->current.pitch_out = angle_to_10000(pilota_quad->current.pitch_f);

			//printf("throttle_out %d yaw_out %d roll_out %d pitch_out %d \n", throttle_out, yaw_out, roll_out, pitch_out);

			// left
			pilota_quad->attuatori.value[1] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
			pilota_quad->attuatori.value[2] = pilota_quad->current.throttle_out - pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

			// right
			pilota_quad->attuatori.value[0] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out - pilota_quad->current.pitch_out; // FRONT
			pilota_quad->attuatori.value[3] = pilota_quad->current.throttle_out + pilota_quad->current.roll_out + pilota_quad->current.pitch_out; // BACK

			// Yaw input
			pilota_quad->attuatori.value[0] += pilota_quad->current.yaw_out; // CCW
			pilota_quad->attuatori.value[1] -= pilota_quad->current.yaw_out; // CW
			pilota_quad->attuatori.value[2] += pilota_quad->current.yaw_out; // CCW
			pilota_quad->attuatori.value[3] -= pilota_quad->current.yaw_out; // CW

			// We need to clip motor output at out_max. When cipping a motors
			// output we also need to compensate for the instability by
			// lowering the opposite motor by the same proportion. This
			// ensures that we retain control when one or more of the motors
			// is at its maximum output
			for (i=0; i<4; i++) {
				if (pilota_quad->attuatori.value[i] > pilota_quad->attuatori.max[i]) {
					// note that i^1 is the opposite motor
					switch(i){
					case 0:
						pilota_quad->attuatori.value[2] -= (pilota_quad->attuatori.value[0] - pilota_quad->attuatori.max[0]);
						pilota_quad->attuatori.value[0] = pilota_quad->attuatori.max[0];
						break;
					case 1:
						pilota_quad->attuatori.value[3] -= (pilota_quad->attuatori.value[1] - pilota_quad->attuatori.max[1]);
						pilota_quad->attuatori.value[1] = pilota_quad->attuatori.max[1];
						break;
					case 2:
						pilota_quad->attuatori.value[0] -= (pilota_quad->attuatori.value[2] - pilota_quad->attuatori.max[2]);
						pilota_quad->attuatori.value[2] = pilota_quad->attuatori.max[2];
						break;
					case 3:
						pilota_quad->attuatori.value[1] -= (pilota_quad->attuatori.value[3] - pilota_quad->attuatori.max[3]);
						pilota_quad->attuatori.value[3] = pilota_quad->attuatori.max[3];
						break;
					}
				}
			}

			// limit output so motors don't stop
			pilota_quad->attuatori.value[0] = max(pilota_quad->attuatori.value[0], pilota_quad->attuatori.min[0] + out_min);
			pilota_quad->attuatori.value[1] = max(pilota_quad->attuatori.value[1], pilota_quad->attuatori.min[1] + out_min);
			pilota_quad->attuatori.value[2] = max(pilota_quad->attuatori.value[2], pilota_quad->attuatori.min[2] + out_min);
			pilota_quad->attuatori.value[3] = max(pilota_quad->attuatori.value[3], pilota_quad->attuatori.min[3] + out_min);

			//printf("0 %d 1 %d 2 %d 3 %d\n",pilota_quad->attuatori.value[0],pilota_quad->attuatori.value[1],pilota_quad->attuatori.value[2],pilota_quad->attuatori.value[3]);

			pilota_quad->last = pilota_quad->current;

		}
		write_pilota();
		usleep(20000);

		//		if((pilota_quad->sensori.gps.altitude > (altezzaEnd - 5)) && (pilota_quad->sensori.gps.altitude < (altezzaEnd + 5))) {
		if(pilota_quad->current.throttle_f <= 0.0001) {
			end = 1;
		}
	} while (end < 1);

	return(0);
}

/////////////////////////// angle_to_10000 //////////////////////////////////
int16_t angle_to_10000(float angle) {
	return (angle * 10000/PI);
}
