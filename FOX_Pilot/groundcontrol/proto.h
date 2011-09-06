/*
 * proto.h
 *
 *  Created on: 03/ago/2011
 *      Author: vito
 */

#ifndef PROTO_H_
#define PROTO_H_

#define MAV_BUFFER_LENGTH 2041

#include <groundcontrol.h>

#include <mavlink.h>

void *protocol_loop(void *ptr);
void init_protocol();

//Input
int handle_message(uint8_t *buf, int dim);

//Output
/*Send Heartbeat */
void send_system_state(void);

int send_mav_heartbeat(void);
int send_mav_boot(void);
int send_mav_system_time(void);
int send_mav_ping(uint8_t target_system, uint8_t target_component, uint32_t seq);
int send_mav_system_time_utc(void);
int send_mav_param_value(uint16_t param_id);
int send_mav_status(void);
int send_mav_local_position(void);
int send_mav_global_position(void);
int send_mav_attitude(void);
int send_mav_control_status(void);
int send_mav_action_ack(uint8_t action, uint8_t result);
int send_mav_global_position_int(void);
int send_mav_vfr_hud(void);
int send_mav_command_ack(float command, float result);
int send_mav_statustext(uint8_t severity, uint8_t *text, int dim);
int send_mav_rc_channels_raw(void);
int send_mav_rc_channels_scaled(void);
int send_mav_servo_output_raw(void);

//VARIE
uint64_t microsSinceEpoch();

//VARIE
void start_gyro_calibration(void);
void param_write_all(void);
void param_read_all(void);
void start_pressure_calibration(void);
void gps_set_local_origin(void);
void start_mag_calibration(void);

int64_t sys_time_clock_get_unix_offset(void);
void sys_time_clock_set_unix_offset(int64_t offset);


#endif /* PROTO_H_ */
