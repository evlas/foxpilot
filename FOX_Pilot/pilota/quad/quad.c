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

#include <defines.h>
#include <config.h>

#include "quad.h"
#include <groundcontrol.h>
#include <watchdog.h>
#include <sensori.h>
#include <attuatori.h>
#include <pilota.h>

/*
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// X MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//stabilizza il volo
int processFlightControlQuadX(pilota_t *pilota_quad) {
	if (pilota_quad->groundcontrol.engage) {
		gettimeofday(&(pilota_quad->current.time),NULL);

		altitude_control(pid_t *altitudePID, pilota_t *pilota_quad);


			// Quadcopter mix
			Servo_Timer2_set(RIGHT_M,radio.ch[2] - rollPID.value - yawPID.value);    // Right motor
			Servo_Timer2_set(LEFT_M,radio.ch[2] + rollPID.value - yawPID.value);     // Left motor
			Servo_Timer2_set(FRONT_M,radio.ch[2] + pitchPID.value + yawPID.value);   // Front motor
			Servo_Timer2_set(BACK_M,radio.ch[2] - pitchPID.value + yawPID.value);    // Back motor
		}
	} else {
		gettimeofday(&(pilota_quad->new.time),NULL);
	}

	return(0);
}

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlQuadPlus(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();

  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motors.setMotorCommand(FRONT, throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR, throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT, throttle - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT, throttle + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
  }

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    processHardManuevers();
  }

  // Apply limits to motor commands
  for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
    motors.setMotorCommand(motor, constrain(motors.getMotorCommand(motor), motors.getMinCommand(motor), motors.getMaxCommand(motor)));
  }

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
 */

/* ************************************************************ */
/* Altitude control */
/*
void altitude_control(pid_t *altitudePID, pilota_t *pilota_quad) {
	double G_dt;

	G_dt = difftimeval(pilota_quad->current.time, pilota_quad->last.time);

	altitudePID->err_old = altitudePID->err;
	altitudePID->err = pilota_quad->groundcontrol.altitude - pilota_quad->sensori.gps.altitude;
	altitudePID->err = constrain_float(altitudePID.err,-60,60);
	altitudePID.D = (float)(altitudePID.err-altitudePID.err_old)/G_dt;
	altitudePID.I += (float)altitudePID.err*G_Dt;
	altitudePID.I = constrain_float(altitudePID.I,-150,150);
	altitudePID.value = Initial_Throttle + altitudePID.KP*altitudePID.err + altitudePID.KD*altitudePID.D + altitudePID.KI*altitudePID.I;
}
*/
/* ************************************************************ */
// ROLL, PITCH and YAW PID controls...
/*
void Attitude_control(pid_t *rollPID, pid_t *pitchPID, pid_t *yawPID) {
	// ROLL CONTROL
	if (pilot.mode==1) {
		//		command_rx_roll += command_RF_roll;     // Add position control term
		command_rx_roll_diff = 0;
	}

	rollPID.err = command_rx_roll - ToDeg(roll);

	rollPID.err = constrain_float(rollPID.err,-25,25);  // to limit max roll command...

	rollPID.I += rollPID.err*G_Dt;
	rollPID.I = constrain_float(rollPID.I,-20,20);
	// D term implementation => two parts: gyro part and command part
	// To have a better (faster) response we can use the Gyro reading directly for the Derivative term...
	// Omega[] is the raw gyro reading plus Omega_I, so it´s bias corrected
	// We also add a part that takes into account the command from user (stick) to make the system more responsive to user inputs
	rollPID.D = command_rx_roll_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[0]);  // Take into account Angular velocity of the stick (command)

	// PID control
	rollPID.value = rollPID.KP*rollPID.err + rollPID.KD*rollPID.D + rollPID.KI*rollPID.I;

	// PITCH CONTROL
	if (pilot.mode==1) {
		//		command_rx_pitch += command_RF_pitch;     // Add position control term
		command_rx_pitch_diff = 0;
	}

	pitchPID.err = command_rx_pitch - ToDeg(pitch);

	pitchPID.err = constrain_float(pitchPID.err,-25,25);  // to limit max pitch command...

	pitchPID.I += pitchPID.err*G_Dt;
	pitchPID.I = constrain_float(pitchPID.I,-20,20);
	// D term
	pitchPID.D = command_rx_pitch_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[1]);

	// PID control
	pitchPID.value = pitchPID.KP*pitchPID.err + pitchPID.KD*pitchPID.D + pitchPID.KI*pitchPID.I;

	// YAW CONTROL
	yawPID.err = command_rx_yaw - ToDeg(yaw);
	if (yawPID.err > 180)    // Normalize to -180,180
		yawPID.err -= 360;
	else if(yawPID.err < -180)
		yawPID.err += 360;

	yawPID.err = constrain_float(yawPID.err,-60,60);  // to limit max yaw command...

	yawPID.I += yawPID.err*G_Dt;
	yawPID.I = constrain_float(yawPID.I,-50,50);
	yawPID.D = command_rx_yaw_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[2]);

	// PID control
	yawPID.value= yawPID.KP*yawPID.err + yawPID.KD*yawPID.D + yawPID.KI*yawPID.I;
}
*/
/*
int channel_filter(int ch, int ch_old) {
	if (ch_old==0) {
		return(ch);
	} else {
		return((ch+ch_old)>>1);     // small filtering (average filter)
	}
}
*/

