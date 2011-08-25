/*
 * sys_state.h
 *
 *  Created on: 15/ago/2011
 *      Author: vito
 */

#ifndef SYS_STATE_H_
#define SYS_STATE_H_

#include <groundcontrol.h>

#include <mavlink.h>

#include "radio.h"

typedef struct __data_stream_t {
	bool enable;
	uint16_t rate;
} data_stream_t;

enum {
	FLY_WAIT_MOTORS,
	FLY_RAMP_UP,//open loop
	FLY_STARTING,
	FLY_FLYING,
	FLY_SINKING,//4
	FLY_WAIT_LANDING,//5
	FLY_LANDING,//open loop 6
	FLY_RAMP_DOWN,//open loop 7
	FLY_GROUNDED// 8
} fly_state_id;

/*
typedef struct {
	uint8_t position_fix;
	uint8_t fly;
	uint8_t attitude_control_enabled;
	uint8_t position_xy_control_enabled;
	uint8_t position_z_control_enabled;
	uint8_t position_yaw_control_enabled;
	//todo use these instead of params
	float mix_remote;
	float mix_position;
	float mix_z_position;
	float mix_yaw_position;
	float mix_offset;
	enum MAV_MODE mav_mode; //OK
	enum MAV_NAV nav_mode; //OK
	enum MAV_TYPE type; //OK
	enum MAV_STATE status; //OK
	uint8_t gps_mode;//< comes from parameter for faster check
	uint8_t uart0mode;
	uint8_t uart1mode;
} sys_state_t;
 */
typedef struct {
	enum MAV_MODE mav_mode;
	enum MAV_NAV nav_mode;
	enum MAV_TYPE type;
	enum MAV_AUTOPILOT_TYPE autopilot;
	enum MAV_STATE status;
	enum MAV_STATE prevstatus;

	bool failsafe;
	bool indoor;

	uint8_t position_fix;
	uint8_t fly;

	manual_ctrl_t remote;
} sys_state_t;

void set_datastream(enum MAV_DATA_STREAM dstream, uint16_t rate, bool enable);

/** @return Get the current mode of operation */
enum MAV_MODE get_sys_state_mode(void);
/** @brief Set the current mode of operation */
bool set_sys_state_mode(enum MAV_MODE mode);

/** @brief Get the current navigation mode */
enum MAV_NAV get_sys_state_nav_mode(void);
/** @brief Set the current navigation mode */
void set_sys_state_nav_mode(enum MAV_NAV nav_mode);

/** @brief Get the current system type */
enum MAV_TYPE get_sys_state_type(void);
/** @brief Set the current system type */
void set_sys_state_type(enum MAV_TYPE type);

/** @brief Get the current autopilot type */
enum MAV_AUTOPILOT_TYPE get_sys_state_autopilot(void);
/** @brief Set the current autopilot type */
void set_sys_state_autopilot(enum MAV_AUTOPILOT_TYPE type);

/** @brief Get the current system state */
enum MAV_STATE get_sys_state_status(void);
/** @brief Set the current system state */
bool set_sys_state_status(enum MAV_STATE state);

uint8_t get_sys_state_position_fix(void);
void set_sys_state_position_fix(uint8_t position_fix);

/** @brief Check if the system is currently in flight mode */
bool get_sys_state_is_flying(void);
uint8_t get_sys_state_fly(void);
void set_sys_state_fly(uint8_t fly);

/** @brief Check if the system is indoor flight mode */
bool get_sys_state_is_indoor(void);

/** @brief Get the current manual crtl */
manual_ctrl_t get_sys_state_manual_ctrl(void);
/** @brief Set the current manual crtl */
bool set_sys_state_manual_ctrl(manual_ctrl_t mctrl);

//////////////////////////////////////////////////////////////////////////

uint16_t get_peak_cpu_load(uint64_t loop_start_time,
		uint64_t loop_stop_time, uint64_t min_mainloop);
uint16_t get_avg_cpu_load(uint64_t loop_start_time,
		uint64_t loop_stop_time, uint64_t min_mainloop);

void update_system_statemachine();

#endif /* SYS_STATE_H_ */
