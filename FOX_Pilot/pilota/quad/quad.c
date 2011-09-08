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
int processFlightControlQuadX(pilota_t *pilota_quad) {
	int i;
	int out_min = 500;	//5%

	if (pilota_quad->groundcontrol.state.status > MAV_STATE_STANDBY) {
		processAttitude(pilota_quad);

		processHeading(pilota_quad);

		processThrottle(pilota_quad);

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

}

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int processFlightControlQuadPlus(pilota_t *pilota_quad) {
}

int processFlightControlQuadPlusMaual(pilota_t *pilota_quad) {
}

int processFlightControlQuadPlusAuto(pilota_t *pilota_quad) {
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAttitude //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAttitude(pilota_t *pilota_quad) {
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

	pilota_quad->current.roll_f = clamp(pilota_quad->current.roll_f,-0.4,0.4);
	pilota_quad->current.pitch_f = clamp(pilota_quad->current.pitch_f,-0.4,0.4);

	printf("pilota_quad->current.roll_f %f\n", pilota_quad->current.roll_f);
	printf("pilota_quad->current.pitch_f %f\n", pilota_quad->current.pitch_f);
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeading ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading(pilota_t *pilota_quad) {
	float yaw_incr;

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

	pilota_quad->current.yaw_f = clamp(pilota_quad->current.yaw_f,-0.5,0.5);
	pilota_quad->current.heading = yaw_incr;

	printf("pilota_quad->current.yaw_f %f\n", pilota_quad->current.yaw_f);

	//commandedYaw = constrain(receiver.getSIData(YAW) + radians(headingHold), -PI, PI);
	//motors.setMotorAxisCommand(YAW, updatePID(commandedYaw, gyro.getData(YAW), &PID[YAW]));
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processThrottle //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processThrottle(pilota_t *pilota_quad) {
	pilota_quad->current.throttle_f = pilota_quad->groundcontrol.state.remote.thrust *
			(2.0 - (cos(pilota_quad->current.roll_f)*cos(pilota_quad->current.pitch_f)));

	pilota_quad->current.throttle_f = clamp(pilota_quad->current.throttle_f,0.0,1.0);

	printf("pilota_quad->current.throttle_f %f\r\r\r\r", pilota_quad->current.throttle_f);
}


//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAltitude //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAltitude(pilota_t *pilota_quad) {

}







/////////////////////////// angle_to_10000 //////////////////////////////////
int16_t angle_to_10000(float angle) {
	return (angle * 10000/PI);
}
