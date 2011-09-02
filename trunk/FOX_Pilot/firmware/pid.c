/*
 * pid.c
 *
 *  Created on: Sep 11, 2009
 *      Author: pixhawk
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <math/pid.h>

/**
 *
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 * @param intmax set to 0 to disable integral windup prevention
 */
void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax, uint8_t mode, uint8_t plot_i) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	pid->mode = mode;
	pid->plot_i = plot_i;
	pid->count = 0;
	pid->saturated = 0;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;
}

void pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	//	pid->mode = mode;

	//	pid->sp = 0;
	//	pid->error_previous = 0;
	//	pid->integral = 0;
}

//void pid_set(PID_t *pid, float sp)
//{
//	pid->sp = sp;
//	pid->error_previous = 0;
//	pid->integral = 0;
//}

/**
 *
 * @param pid
 * @param sp	targetPosition
 * @param val	currentPosition
 * @param dt
 * @return
 */
float pid_calculate(PID_t *pid, float sp, float val, float val_dot) {
	/*  error = setpoint - actual_position
	 integral = integral + (error*dt)
	 derivative = (error - previous_error)/dt
	 output = (Kp*error) + (Ki*integral) + (Kd*derivative)
	 previous_error = error
	 wait(dt)
	 goto start
	 */
	uint64_t currentTime = microsSinceEpoch();
	float dt = (currentTime - pid->previousTime) / 1000000.0;;

	float i, d;
	pid->sp = sp;
	float error = pid->sp - val;

	if (pid->saturated && (pid->integral * error > 0)) {
		//Output is saturated and the integral would get bigger (positive or negative)
		i = pid->integral;

		//Reset saturation. If we are still saturated this will be set again at output limit check.
		pid->saturated=0;
	} else {
		i = pid->integral + (error * dt);
	}
	// Anti-Windup. Needed if we don't use the saturation above.
	if (pid->intmax != 0.0) {
		if (i > pid->intmax) {
			pid->integral = pid->intmax;
		} else if (i < -pid->intmax) {
			pid->integral = -pid->intmax;
		} else {
			pid->integral = i;
		}
	}

	if (pid->mode == PID_MODE_DERIVATIV_CALC) {
		d = (error - pid->error_previous) / dt;
	} else if (pid->mode == PID_MODE_DERIVATIV_SET) {
		d = -val_dot;
	} else {
		d = 0;
	}

	pid->error_previous = error;
	pid->previousTime = currentTime;

	return ((error * pid->kp) + (i * pid->ki) + (d * pid->kd));
}
