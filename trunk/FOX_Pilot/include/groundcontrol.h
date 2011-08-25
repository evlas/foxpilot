/*
 * groundcontrol.h
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#ifndef GROUNDCONTROL_H_
#define GROUNDCONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>

#include <groundcontrol/proto.h>
#include <groundcontrol/param.h>
#include <groundcontrol/sys_state.h>
/*
struct global_struct {
	uint32_t pressure_raw;                    ///< Raw ambient pressure, in ADC units
	uint32_t pressure_si;					  ///< Pressure in Pascal
	uint32_t pressure_diff_raw;				  ///< Raw differential pressure in ADC units
	uint32_t pressure_diff_si;				  ///< Differential pressure in Pascal
	uint16_vect3 gyros_raw;                   ///< gyros raw sensor data
// gyros raw sensor data offset
//	uint16_vect3 gyros_raw_offset;
	int16_vect3 accel_raw;                    ///< accel raw sensor data
	int16_vect3 magnet_raw;                   ///< magnetometer 	PARAM_IMU_RESET,

	PARAM_UART0_BAUD,
	PARAM_UART1_BAUD,
	PARAM_UART2_BAUD,
	PARAM_UART3_BAUD,
	PARAM_UART4_BAUD,

	PARAM_PID_ATT_P,
	PARAM_PID_ATT_I,
	PARAM_PID_ATT_D,
	PARAM_PID_ATT_LIM,
	PARAM_PID_ATT_AWU,

	PARAM_PID_POS_P,
	PARAM_PID_POS_I,
	PARAM_PID_POS_D,
	PARAM_PID_POS_LIM,
	PARAM_PID_POS_AWU,

	///raw sensor data
	uint16_t supersonic_raw;                  ///< supersonic raw sensor data
// RAW adc data of the gyro temperature sensor
	uint16_t temperature_gyros;               ///<
// temperature sensor data
	int16_t temperature;                      ///<
	float temperature_si;                     ///< Temperature in degrees celsius

	/// Calibration values and zero offsets
	int16_vect3 gyros_autocal_raw_offset;     ///< Zero-speed offsets of gyros, calibrated at startup

	/// Measurements in physical SI units (http://en.wikipedia.org/wiki/International_System_of_Units)
	///
	float_vect3 gyros_si;                     ///< Angular speed in rad/s
	float_vect3 accel_si;                     ///< Linear acceleration in body frame in m/s^2
	int16_vect3 magnet_corrected;	  		  ///< Magnet Sensor data with corrected offset (raw values)

	/// System state representation
	float_vect3 attitude;                     ///< Angular position / attitude in Tait-Bryan angles (http://en.wikipedia.org/wiki/Yaw,_pitch,_and_roll)
	float_vect3 velocity;                        ///< Current speed of MAV in m/s
	float_vect3 position;                     ///< vector from origin to mav in body coordinates
	float_vect3 position_setpoint;            ///<
	float yaw_pos_setpoint;                   ///<


	float_vect3 position_setpoint_min;        ///<
	float_vect3 position_setpoint_max;        ///<
	float_vect3 position_raw;                 ///<
	float_vect3 position_control_output;      ///< Output of position controller
	float       position_yaw_control_output;
	float_vect3 attitude_control_output;      ///< Output of attitude controller
	float       thrust_control_output;        ///< Output of thrust controller
	float_vect3 attitude_setpoint_pos;        ///< (from position controller)
	float_vect3 attitude_setpoint_pos_body;   ///< In body frame
	float_vect3 attitude_setpoint_pos_body_offset; ///< Additional offset to attitude body control output
	float_vect3 attitude_setpoint_remote;     ///< (from remote)
	float_vect3 attitude_setpoint_offset;     ///< (not aligned IMU to frame, not eliminated sensor offset)
	float_vect3 attitude_setpoint;            ///< angles for the attitude controller

	float gas_remote;
// position error in body coordinates
	float_vect3 position_error;
	sys_state_t state;                        ///< Current vehicle state representation
	uint8_t motor_block;                      ///< Position of motor block switch
	comm_state_t comm;                        ///< Packet drop rate of receiving channels
	uint16_t pwm_values[PWM_NB_CHANNELS];     ///< Servo outputs
	uint16_t ppm_values[PPM_NB_CHANNEL];      ///< RC inputs
	uint32_t watchdog_error;
	uint16_t battery_voltage;                 ///< Battery voltage in mV
	uint16_t cpu_usage;                       ///< CPU usage, 0 = 0%, 1000 = 100%
	uint16_t cpu_peak;                       ///< CPU peak, 0 = 0%, 1000 = 100%
// PID state variables
	PID_t pid_fx, pid_fy, pid_fz, pid_yaw, pid_yawspeed;
// Global position setpoint
	float_vect3 reference_world;
	float param[ONBOARD_PARAM_COUNT];         ///< EEPROM parameter values
	char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];  ///< EEPROM parameter names
	float ground_distance;
	float ground_distance_unfiltered;
	float_vect3 vicon_data;
	vision_t vision_data;                     ///< Data from computer vision system
	uint64_t pos_last_valid;
	uint64_t vicon_last_valid;
	uint64_t entry_critical;

	uint16_t i2c0_err_count;                  ///< I2C0 errors
	uint16_t i2c1_err_count;                  ///< I2C1 errors
	uint16_t spi_err_count;                   ///< SPI errors

	float thrust_hover_offset;
	float thrust_calibration_increment;
	float thrust_landing;
	bool waiting_over;
	bool ramp_up;
	float motor_thrust_actual;

	uint8_t rc_rssi;							  ///< Receive Signal Strength Indicator (0: 0%, 255: 100%)
	uint16_t rc_chan_cal_min[9];
	uint16_t rc_chan_cal_center[9];
	uint16_t rc_chan_cal_max[9];
};
 */
typedef struct __groundcontrol_t {
	sys_state_t state;                        ///< Current vehicle state representation
	uint16_t cpu_usage;                       ///< CPU usage, 0 = 0%, 1000 = 100%
	uint16_t cpu_peak;                        ///< CPU peak, 0 = 0%, 1000 = 100%
//	uint8_t motor_block;                      ///< Position of motor block switch
	comm_t comm;
	float param[ONBOARD_PARAM_COUNT];
	char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];
	data_stream_t datastream[MAV_DATA_STREAM_ENUM_END];
} groundcontrol_t;

groundcontrol_t groundcontrol_data;
pthread_mutex_t groundcontrol_mutex;

void init_groundcontrol();
void deinit_groundcontrol();

void param_defaults_groundcontrol(void);

int read_groundcontrol(groundcontrol_t *a);
int write_groundcontrol(groundcontrol_t *a);

void set_failsafe_groundcontrol(bool failsafe);
bool get_failsafe_groundcontrol();

//int write_mode_groundcontrol(uint8_t mode);

#endif /* GROUNDCONTROL_H_ */
