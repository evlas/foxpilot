#include <math/mav_vect.h>
#include <math/pid.h>
#include <string.h>
//#include <mavlink_types.h>
#include <mavlink.h>

#include <defines.h>
#include <config.h>

#ifndef GLOBAL_DATA_H_
#define GLOBAL_DATA_H_

// TODO Move
#define MOTORS_BLOCKED 0
#define MOTORS_ENABLED 1







enum uartmodes {
	UART_MODE_MAVLINK = 0,
	UART_MODE_GPS = 1,
	UART_MODE_BYTE_FORWARD = 2
};

typedef struct {
	float_vect3 pos;
	float_vect3 vel;
	float_vect3 ang;

	float confidence;
	uint64_t time_captured;
	uint64_t comp_end;

	int new_data; /* New data available = 1; No new Data = 0 */
} vision_t;



/*
typedef struct {
	uint32_t uart0_rx_drop_count;     /// UART0 Receive drops
	uint32_t uart0_rx_success_count;  /// UART0 Receive successes
	uint32_t uart1_rx_drop_count;     /// UART0 Receive drops
	uint32_t uart1_rx_success_count;  /// UART0 Receive successes
} comm_state_t;
 */


/**
 * @brief reset all parameters to default
 * @warning DO NOT USE THIS IN FLIGHT!
 */
/*
static inline void global_data_reset_param_defaults(void) {

	global_data.param[PARAM_SYSTEM_ID] = 1;
	strcpy(global_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");

	global_data.param[PARAM_COMPONENT_ID] = 200;
	strcpy(global_data.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");

	global_data.param[PARAM_SYSTEM_TYPE] = MAV_QUADROTOR;
	strcpy(global_data.param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");

	global_data.param[PARAM_SW_VERSION] = 2000;
	strcpy(global_data.param_name[PARAM_SW_VERSION], "SYS_SW_VER");

	global_data.param[PARAM_PPM_SAFETY_SWITCH_CHANNEL] = 5;
	strcpy(global_data.param_name[PARAM_PPM_SAFETY_SWITCH_CHANNEL], "RC_SAFETY_CHAN");

	global_data.param[PARAM_UART0_BAUD] = 115200;//57600;//
	strcpy(global_data.param_name[PARAM_UART0_BAUD], "UART_0_BAUD");

	global_data.param[PARAM_UART1_BAUD] = 57600;//57600
	strcpy(global_data.param_name[PARAM_UART1_BAUD], "UART_1_BAUD");

	global_data.param[PARAM_UART1_BAUD] = 57600;//57600
	strcpy(global_data.param_name[PARAM_UART2_BAUD], "UART_2_BAUD");

	global_data.param[PARAM_UART1_BAUD] = 57600;//57600
	strcpy(global_data.param_name[PARAM_UART3_BAUD], "UART_3_BAUD");

	global_data.param[PARAM_UART1_BAUD] = 57600;//57600
	strcpy(global_data.param_name[PARAM_UART4_BAUD], "UART_4_BAUD");

	global_data.param[PARAM_PPM_TUNE1_CHANNEL] = 7;
	strcpy(global_data.param_name[PARAM_PPM_TUNE1_CHANNEL], "RC_TUNE_CHAN1");
	global_data.param[PARAM_PPM_TUNE2_CHANNEL] = 5;
	strcpy(global_data.param_name[PARAM_PPM_TUNE2_CHANNEL], "RC_TUNE_CHAN2");
	global_data.param[PARAM_PPM_TUNE3_CHANNEL] = 6;
	strcpy(global_data.param_name[PARAM_PPM_TUNE3_CHANNEL], "RC_TUNE_CHAN3");
	global_data.param[PARAM_PPM_TUNE4_CHANNEL] = 8;
	strcpy(global_data.param_name[PARAM_PPM_TUNE4_CHANNEL], "RC_TUNE_CHAN4");

	global_data.param[PARAM_PPM_THROTTLE_CHANNEL] = 3;
	strcpy(global_data.param_name[PARAM_PPM_THROTTLE_CHANNEL], "RC_THRUST_CHAN");

	global_data.param[PARAM_PPM_YAW_CHANNEL] = 4;
	strcpy(global_data.param_name[PARAM_PPM_YAW_CHANNEL], "RC_YAW_CHAN");

	global_data.param[PARAM_PPM_ROLL_CHANNEL] = 2;
	strcpy(global_data.param_name[PARAM_PPM_ROLL_CHANNEL], "RC_ROLL_CHAN");

	global_data.param[PARAM_TRIMCHAN] = 0;
	strcpy(global_data.param_name[PARAM_TRIMCHAN], "RC_TRIM_CHAN");

	global_data.param[PARAM_PPM_NICK_CHANNEL] = 1;
	strcpy(global_data.param_name[PARAM_PPM_NICK_CHANNEL], "RC_NICK_CHAN");

	global_data.param[PARAM_CAM_INTERVAL] = 36000;//32000;
	strcpy(global_data.param_name[PARAM_CAM_INTERVAL], "CAM_INTERVAL");
	global_data.param[PARAM_CAM_EXP] = 1000;
	strcpy(global_data.param_name[PARAM_CAM_EXP], "CAM_EXP");

	global_data.param[PARAM_POSITIONSETPOINT_ACCEPT] = 0;
	strcpy(global_data.param_name[PARAM_POSITIONSETPOINT_ACCEPT], "POS_SP_ACCEPT");

	global_data.param[PARAM_POSITION_TIMEOUT] = 2000000;
	strcpy(global_data.param_name[PARAM_POSITION_TIMEOUT], "POS_TIMEOUT");

	global_data.param[PARAM_POSITION_SETPOINT_X] =1.1f;//0.21;// 0.5;
	global_data.param[PARAM_POSITION_SETPOINT_Y] =1.1f;//0.145;// 0.44;
	global_data.param[PARAM_POSITION_SETPOINT_Z] = -0.8f;//-1.0;
	global_data.param[PARAM_POSITION_SETPOINT_YAW] = 0.0f;

	global_data.param[PARAM_POSITION_YAW_TRACKING]=0;

	global_data.attitude_setpoint_pos_body_offset.x = 0.0f;
	global_data.attitude_setpoint_pos_body_offset.y = 0.0f;
	global_data.attitude_setpoint_pos_body_offset.z = 0.0f;

	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_X], "POS_SP_X");
	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_Y], "POS_SP_Y");
	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_Z], "POS_SP_Z");
	strcpy(global_data.param_name[PARAM_POSITION_SETPOINT_YAW], "POS_SP_YAW");
	strcpy(global_data.param_name[PARAM_POSITION_YAW_TRACKING], "POS_YAW_TRACK");


	//	// FOR FLYING WITHOUT CABLE
	//		global_data.param[PARAM_PID_ATT_P] = 45.3;
	//		strcpy(global_data.param_name[PARAM_PID_ATT_P], "PID_ATT_P");
	//		global_data.param[PARAM_PID_ATT_I] = 0;//0
	//		strcpy(global_data.param_name[PARAM_PID_ATT_I], "PID_ATT_I");
	//		global_data.param[PARAM_PID_ATT_D] = 14.9;
	//		strcpy(global_data.param_name[PARAM_PID_ATT_D], "PID_ATT_D");
	//		global_data.param[PARAM_PID_ATT_AWU] = 1;//1
	//		strcpy(global_data.param_name[PARAM_PID_ATT_AWU], "PID_ATT_AWU");

// FOR FLYING WITH CABLE
	global_data.param[PARAM_PID_ATT_P] = 90; // 45 Bravo
	strcpy(global_data.param_name[PARAM_PID_ATT_P], "PID_ATT_P");
	global_data.param[PARAM_PID_ATT_I] = 60; // 15 Bravo
	strcpy(global_data.param_name[PARAM_PID_ATT_I], "PID_ATT_I");
	global_data.param[PARAM_PID_ATT_D] = 30; // 15 Bravo
	strcpy(global_data.param_name[PARAM_PID_ATT_D], "PID_ATT_D");
	global_data.param[PARAM_PID_ATT_LIM] = 100;//not yet used!!!!
	strcpy(global_data.param_name[PARAM_PID_ATT_LIM], "PID_ATT_LIM");
	global_data.param[PARAM_PID_ATT_AWU] = 0.3;//1
	strcpy(global_data.param_name[PARAM_PID_ATT_AWU], "PID_ATT_AWU");

	global_data.param[PARAM_PID_YAWPOS_P] = 5;//1;
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_P], "PID_YAWPOS_P");
	global_data.param[PARAM_PID_YAWPOS_I] = 0.1;//0
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_I], "PID_YAWPOS_I");
	global_data.param[PARAM_PID_YAWPOS_D] = 1;//0.5;
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_D], "PID_YAWPOS_D");
	global_data.param[PARAM_PID_YAWPOS_LIM] = 3;
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_LIM], "PID_YAWPOS_LIM");
	global_data.param[PARAM_PID_YAWPOS_AWU] = 1;//1
	strcpy(global_data.param_name[PARAM_PID_YAWPOS_AWU], "PID_YAWPOS_AWU");

	global_data.param[PARAM_PID_YAWSPEED_P] = 15;
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_P], "PID_YAWSPEED_P");
	global_data.param[PARAM_PID_YAWSPEED_I] = 5;//0
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_I], "PID_YAWSPEED_I");
	global_data.param[PARAM_PID_YAWSPEED_D] = 0;
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_D], "PID_YAWSPEED_D");
	global_data.param[PARAM_PID_YAWSPEED_LIM] = 50;//not yet used!!!!
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_LIM], "PID_YAWSPE_LIM");
	global_data.param[PARAM_PID_YAWSPEED_AWU] = 1;//1
	strcpy(global_data.param_name[PARAM_PID_YAWSPEED_AWU], "PID_YAWSPE_AWU");

// Position
	global_data.param[PARAM_PID_POS_P] =1.8;// 1.6f;  //0.5; //2.4;//5
	strcpy(global_data.param_name[PARAM_PID_POS_P], "PID_POS_P");
	global_data.param[PARAM_PID_POS_I] =0.2;// 0.35f; //0.3;//0.1
	strcpy(global_data.param_name[PARAM_PID_POS_I], "PID_POS_I");
	global_data.param[PARAM_PID_POS_D] = 2.0f;  //0.5;//1.6;//1
	strcpy(global_data.param_name[PARAM_PID_POS_D], "PID_POS_D");
	global_data.param[PARAM_PID_POS_LIM] = 0.2;
	strcpy(global_data.param_name[PARAM_PID_POS_LIM], "PID_POS_LIM");
	global_data.param[PARAM_PID_POS_AWU] = 5;//1
	strcpy(global_data.param_name[PARAM_PID_POS_AWU], "PID_POS_AWU");

	global_data.param[PARAM_PID_POS_Z_P] = 0.5;
	strcpy(global_data.param_name[PARAM_PID_POS_Z_P], "PID_POS_Z_P");
	global_data.param[PARAM_PID_POS_Z_I] = 0.3;//0
	strcpy(global_data.param_name[PARAM_PID_POS_Z_I], "PID_POS_Z_I");
	global_data.param[PARAM_PID_POS_Z_D] = 0.2;
	strcpy(global_data.param_name[PARAM_PID_POS_Z_D], "PID_POS_Z_D");
	global_data.param[PARAM_PID_POS_Z_LIM] = 0.30;//1
	strcpy(global_data.param_name[PARAM_PID_POS_Z_LIM], "PID_POS_Z_LIM");
	global_data.param[PARAM_PID_POS_Z_AWU] = 3;//1
	strcpy(global_data.param_name[PARAM_PID_POS_Z_AWU], "PID_POS_Z_AWU");


	global_data.param[PARAM_GYRO_OFFSET_X] = 29760;//29777;//80
	strcpy(global_data.param_name[PARAM_GYRO_OFFSET_X], "CAL_GYRO_X");
	global_data.param[PARAM_GYRO_OFFSET_Y] = 29860;//29835;//29849;
	strcpy(global_data.param_name[PARAM_GYRO_OFFSET_Y], "CAL_GYRO_Y");
	global_data.param[PARAM_GYRO_OFFSET_Z] = 29877;//29880;//29898;
	strcpy(global_data.param_name[PARAM_GYRO_OFFSET_Z], "CAL_GYRO_Z");
	global_data.param[PARAM_CAL_TEMP] = 32.0f;//29880;//29898;
	strcpy(global_data.param_name[PARAM_CAL_TEMP], "CAL_TEMP");

// Magnetometer offset calibration
	strcpy(global_data.param_name[PARAM_CAL_MAG_OFFSET_X], "CAL_MAG_X");
	global_data.param[PARAM_CAL_MAG_OFFSET_X] = 0;
	strcpy(global_data.param_name[PARAM_CAL_MAG_OFFSET_Y], "CAL_MAG_Y");
	global_data.param[PARAM_CAL_MAG_OFFSET_Y] = 0;
	strcpy(global_data.param_name[PARAM_CAL_MAG_OFFSET_Z], "CAL_MAG_Z");
	global_data.param[PARAM_CAL_MAG_OFFSET_Z] = 0;


	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_X] =  3.149677908834623E+04;
	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Y] =  2.938336621296374e+04;
	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Z] = 3.015110968108719e+04;
	global_data.param[PARAM_CAL_GYRO_TEMP_FIT_ACTIVE] = 0;

	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_X], "CAL_FIT_GYRO_X");
	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_Y], "CAL_FIT_GYRO_Y");
	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_Z], "CAL_FIT_GYRO_Z");
	strcpy(global_data.param_name[PARAM_CAL_GYRO_TEMP_FIT_ACTIVE], "CAL_FIT_ACTIVE");

	global_data.param[PARAM_CAL_PRES_DIFF_OFFSET] = 10000;
	strcpy(global_data.param_name[PARAM_CAL_PRES_DIFF_OFFSET], "CAL_PRES_DIFF");

	global_data.param[PARAM_AC			if (mavlink_msg_set_nav_mode_get_target(&msg) == airplane.system_id) {
				printf("MAVLINK_MSG_ID_SET_NAV_MODE\n");
			}
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			mavlink_param_request_read_t set;
			mavlink_msg_param_request_read_decode(msg, &set);

			// Check if this message is for this system
			if (((uint8_t) set.target_system == (uint8_t) global_data.param[PARAM_SYSTEM_ID])
					&& ((uint8_t) set.target_component == (uint8_t) global_data.param[PARAM_COMPONENT_ID])) {
				char* key = (char*) set.param_id;

				if (set.param_id[0] == '\0') {
					// Choose parameter based on index
					if (set.param_index < ONBOARD_PARAM_COUNT) {
						// Report back value
						mavlink_msg_param_value_send(chan,
								(int8_t*) global_data.param_name[set.param_index],
								global_data.param[set.param_index], ONBOARD_PARAM_COUNT, set.param_index);
					}
				}
			} else {
				for (int i = 0; i < ONBOARD_PARAM_COUNT; i++) {
					bool match = true;
					for (int j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++) {
						// Compare
						if (((char) (global_data.param_name[i][j]))	!= (char) (key[j])) {
							match = false;
						}

						// End matching if null termination is reached
						if (((char) global_data.param_name[i][j]) == '\0') {
							break;
						}
					}

					// Check if matched
					if (match) {
						// Report back value
						mavlink_msg_param_value_send(chan, 	(int8_t*) global_data.param_name[i], global_data.param[i], ONBOARD_PARAM_COUNT, m_parameter_i);
					}
				}
			}
		}
		break;C_OFFSET_X] = 0;//-13;
	strcpy(global_data.param_name[PARAM_ACC_OFFSET_X], "CAL_ACC_X");
	global_data.param[PARAM_ACC_OFFSET_Y] = 0;//13;
	strcpy(global_data.param_name[PARAM_ACC_OFFSET_Y], "CAL_ACC_Y");
	global_data.param[PARAM_ACC_OFFSET_Z] = 0;
	strcpy(global_data.param_name[PARAM_ACC_OFFSET_Z], "CAL_ACC_Z");

	global_data.param[PARAM_ACC_NAVI_OFFSET_X] = 0;//-8;
	strcpy(global_data.param_name[PARAM_ACC_NAVI_OFFSET_X], "ACC_NAV_OFFS_X");
	global_data.param[PARAM_ACC_NAVI_OFFSET_Y] = 0;// 12;
	strcpy(global_data.param_name[PARAM_ACC_NAVI_OFFSET_Y], "ACC_NAV_OFFS_Y");
	global_data.param[PARAM_ACC_NAVI_OFFSET_Z] = -1000;
	strcpy(global_data.param_name[PARAM_ACC_NAVI_OFFSET_Z], "ACC_NAV_OFFS_Z");

	global_data.param[PARAM_VEL_OFFSET_X] = 0;//-0.015;
	strcpy(global_data.param_name[PARAM_VEL_OFFSET_X], "VEL_OFFSET_X");
	global_data.param[PARAM_VEL_OFFSET_Y] = 0;//0.011;
	strcpy(global_data.param_name[PARAM_VEL_OFFSET_Y], "VEL_OFFSET_Y");
	global_data.param[PARAM_VEL_OFFSET_Z] = 0;
	strcpy(global_data.param_name[PARAM_VEL_OFFSET_Z], "VEL_OFFSET_Z");

	global_data.param[PARAM_VEL_DAMP] = 0.95;
	strcpy(global_data.param_name[PARAM_VEL_DAMP], "VEL_DAMP");

	global_data.param[PARAM_ATT_KAL_KACC] = 0.0033;
	strcpy(global_data.param_name[PARAM_ATT_KAL_KACC], "ATT_KAL_KACC");

	//We don't have Magnetometer therfore we integrate yaw gyro between vision data
	global_data.param[PARAM_ATT_KAL_IYAW] = 1;
	strcpy(global_data.param_name[PARAM_ATT_KAL_IYAW], "ATT_KAL_IYAW");

	global_data.param[PARAM_ATT_OFFSET_X] =0;// -0.08;//-0.11;
	strcpy(global_data.param_name[PARAM_ATT_OFFSET_X], "ATT_OFFSET_X");
	global_data.param[PARAM_ATT_OFFSET_Y] =0;// -0.080;//0.085;
	strcpy(global_data.param_name[PARAM_ATT_OFFSET_Y], "ATT_OFFSET_Y");
	global_data.param[PARAM_ATT_OFFSET_Z] = -0.080;//-0.08;
	strcpy(global_data.param_name[PARAM_ATT_OFFSET_Z], "ATT_OFFSET_Z");

	global_data.param[PARAM_SEND_DEBUGCHAN] = MAVLINK_COMM_0;
	strcpy(global_data.param_name[PARAM_SEND_DEBUGCHAN], "SEND_DEBUGCHAN");

	// Always send attitude - this is a core system information
	global_data.param[PARAM_SEND_SLOT_ATTITUDE] = 1;
	global_data.param[PARAM_SEND_SLOT_RAW_IMU] = 0;
	global_data.param[PARAM_SEND_SLOT_REMOTE_CONTROL] = 0;
	global_data.param[PARAM_SEND_SLOT_CONTROLLER_OUTPUT] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_1] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_2] = 0;//1
	global_data.param[PARAM_SEND_SLOT_DEBUG_3] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_4] = 0;//1
	global_data.param[PARAM_SEND_SLOT_DEBUG_5] = 0;
	global_data.param[PARAM_SEND_SLOT_DEBUG_6] = 0;
	strcpy(global_data.param_name[PARAM_SEND_SLOT_ATTITUDE], "SLOT_ATTITUDE");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_RAW_IMU], "SLOT_RAW_IMU");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_REMOTE_CONTROL], "SLOT_RC");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_CONTROLLER_OUTPUT], "SLOT_CONTROL");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_1], "DEBUG_1");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_2], "DEBUG_2");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_3], "DEBUG_3");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_4], "DEBUG_4");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_5], "DEBUG_5");
	strcpy(global_data.param_name[PARAM_SEND_SLOT_DEBUG_6], "DEBUG_6");

	global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
	strcpy(global_data.param_name[PARAM_MIX_REMOTE_WEIGHT], "MIX_REMOTE");
	global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
	strcpy(global_data.param_name[PARAM_MIX_POSITION_WEIGHT], "MIX_POSITION");
	global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
	strcpy(global_data.param_name[PARAM_MIX_POSITION_Z_WEIGHT], "MIX_Z_POSITION");
	global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
	strcpy(global_data.param_name[PARAM_MIX_POSITION_YAW_WEIGHT], "MIX_POS_YAW");
	global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
	strcpy(global_data.param_name[PARAM_MIX_OFFSET_WEIGHT], "MIX_OFFSET");
	global_data.param[PARAM_VISION_YAWCORRECT] = 0;//now done by multitracker //turn by 180 degree 1.57;//turn by 90 degree
	strcpy(global_data.param_name[PARAM_VISION_YAWCORRECT], "VIS_YAWCORR");

	global_data.param[PARAM_VISION_ANG_OUTLAYER_TRESHOLD] = 0.2;
	strcpy(global_data.param_name[PARAM_VISION_ANG_OUTLAYER_TRESHOLD], "VIS_OUTL_TRESH");

	global_data.param[PARAM_VICON_MODE] = 2;
	global_data.param[PARAM_VICON_TAKEOVER_DISTANCE] = 0.5;
	global_data.param[PARAM_VICON_TAKEOVER_TIMEOUT] = 2;
	strcpy(global_data.param_name[PARAM_VICON_MODE], "VICON_MODE");
	strcpy(global_data.param_name[PARAM_VICON_TAKEOVER_DISTANCE], "VICON_TKO_DIST");
	strcpy(global_data.param_name[PARAM_VICON_TAKEOVER_TIMEOUT], "VICON_TKO_TIME");

	strcpy(global_data.param_name[PARAM_GPS_MODE], "GPS_MODE");
	global_data.param[PARAM_GPS_MODE] = 0; //0: MAVLINK, 960010: GPS; // 9600 1 0: 9600 baud, mode 1 = U-Blox binary, on UART 0

	strcpy(global_data.param_name[PARAM_CAM_ANGLE_X_OFFSET], "CAM_ANG_X_OFF");
	strcpy(global_data.param_name[PARAM_CAM_ANGLE_X_FACTOR], "CAM_ANG_X_FAC");
	strcpy(global_data.param_name[PARAM_CAM_ANGLE_Y_OFFSET], "CAM_ANG_Y_OFF");
	strcpy(global_data.param_name[PARAM_CAM_ANGLE_Y_FACTOR], "CAM_ANG_Y_FAC");

	global_data.param[PARAM_CAM_ANGLE_X_OFFSET] = 0;
	global_data.param[PARAM_CAM_ANGLE_X_FACTOR] = 0;
	global_data.param[PARAM_CAM_ANGLE_Y_OFFSET] = 0;
	global_data.param[PARAM_CAM_ANGLE_Y_FACTOR] = 0;

	global_data.param[PARAM_IMU_RESET] = 0;
	strcpy(global_data.param_name[PARAM_IMU_RESET], "SYS_IMU_RESET");
}
*/
/**
 * @brief resets the global data struct to all-zero values
 * @warning DO NOT USE THIS IN FLIGHT!
 */
/*
static inline void global_data_reset(void) {
	global_data.state.mav_mode = MAV_MODE_UNINIT;
	global_data.state.status = MAV_STATE_UNINIT;
	global_data.cpu_usage = 0;
	global_data.cpu_peak = 0;

	global_data.rc_rssi = 0;

	global_data.state.vision_ok=0;
	global_data.state.vicon_ok=0;
	global_data.state.vicon_new_data=0;
	global_data.state.gps_ok=0;
	global_data.state.gps_new_data=0;
	global_data.state.pressure_ok=0;
	global_data.state.remote_ok=0;
	global_data.state.magnet_ok=0;
	global_data.state.ground_distance_ok=0;
	global_data.state.position_fix=0;

	global_data.comm.uart0_rx_drop_count = 0;
	global_data.comm.uart0_rx_success_count = 0;
	global_data.comm.uart1_rx_drop_count = 0;
	global_data.comm.uart1_rx_success_count = 0;

	global_data.ground_distance=0;
	global_data.ground_distance_unfiltered = 0;

	global_data.motor_block = MOTORS_BLOCKED;

	global_data.pos_last_valid = 0; // Make sure there is an overflow in the initial condition
	global_data.vicon_last_valid=0;

	//DONT CHANGE use PARAM!!
	global_data.attitude_setpoint_offset.x = 0.00;
	global_data.attitude_setpoint_offset.y = 0.00;
	global_data.attitude_setpoint_offset.z = 0;

	global_data.yaw_pos_setpoint=0;

	global_data.position_control_output.x = 0.0f;
	global_data.position_control_output.y = 0.0f;
	global_data.position_control_output.z = 0.0f;

	global_data.position_yaw_control_output = 0.0f;

	//safe corridor
	global_data.position_setpoint_min.x=-20;
	global_data.position_setpoint_min.y=-20;
	global_data.position_setpoint_min.z=-1.6;
	global_data.position_setpoint_max.x=20;
	global_data.position_setpoint_max.y=20;
	global_data.position_setpoint_max.z=0;

	// start landing variables initialization
	global_data.thrust_hover_offset = 0.0f;//0.65f;
	global_data.thrust_calibration_increment = 0.00325;//0.00325;//0.065f;//0.0075
	global_data.thrust_landing=0;
	global_data.motor_thrust_actual=0;
	global_data.temperature_si=0;

	//not used anymore?
	global_data.waiting_over = false;
	global_data.ramp_up = false;
}
*/

/* Onboard paramaters / settings */

static inline uint8_t set_parameter(const char *param_id, float value) {
	uint8_t result = 0;
	//	for (int i = 0; i < )
	return (result);
}

static inline uint8_t commit_parameters(void) {
	uint8_t result = 0;
	return (result);
}

static inline uint16_t get_parameter_count() {
	return(0);
}

static inline float get_parameter(uint16_t parameter_id) {}

static inline uint16_t get_parameter_ids(uint16_t *ids, uint16_t len) {
	// Check if enough space is available
	if (len < (ONBOARD_PARAM_COUNT * ONBOARD_PARAM_NAME_LENGTH)) {
		return (0);
	} else {
		return (1);
	}
}

static inline uint8_t get_parameter_names(char *data, uint8_t len) {
	// Check if enough space is available
	if (len < (ONBOARD_PARAM_COUNT * ONBOARD_PARAM_NAME_LENGTH)) {
		return (0);
	} else {
		return (1);
	}
}


#endif /* GLOBAL_DATA_H_ */
