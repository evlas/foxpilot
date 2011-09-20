/*
 * waypoint.h
 *
 *  Created on: 01/set/2011
 *      Author: vito
 */

#include <mavlink.h>
#include <stdbool.h>

#ifndef WAYPOINT_H_
#define WAYPOINT_H_

enum MAVLINK_WPM_STATES
{
    MAVLINK_WPM_STATE_IDLE = 0,
    MAVLINK_WPM_STATE_SENDLIST,
    MAVLINK_WPM_STATE_SENDLIST_SENDWPS,
    MAVLINK_WPM_STATE_GETLIST,
    MAVLINK_WPM_STATE_GETLIST_GETWPS,
    MAVLINK_WPM_STATE_GETLIST_GOTALL,
	MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES
{
    MAVLINK_WPM_CODE_OK = 0,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
    MAVLINK_WPM_CODE_ENUM_END
};

/* WAYPOINT MANAGER - MISSION LIB */
#define MAVLINK_WPM_MAX_WP_COUNT 				100
#define MAVLINK_WPM_SYSTEM_ID 					1
#define MAVLINK_WPM_COMPONENT_ID 				1
#define MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT 	2000000
#define MAVLINK_WPM_SETPOINT_DELAY_DEFAULT 		1000000
#define MAVLINK_WPM_PROTOCOL_DELAY_DEFAULT 		40

typedef struct __mavlink_waypoint_storage_t {
	mavlink_waypoint_t waypoints[MAVLINK_WPM_MAX_WP_COUNT];      ///< Currently active waypoints
	uint16_t size;
	uint16_t max_size;
	uint16_t rcv_size;

	enum MAVLINK_WPM_STATES current_state;

	uint16_t current_wp_id;							///< Waypoint in current transmission (used to trasmit to gcs)
	uint16_t current_active_wp_id;					///< id of current waypoint
	uint16_t current_count;

	uint64_t timestamp_lastaction;
	uint64_t timestamp_last_send_setpoint;
	uint64_t timestamp_firstinside_orbit;			///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value
	uint64_t timestamp_lastoutside_orbit;			///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
	uint32_t timeout;

	uint32_t delay_setpoint;

	float accept_range_yaw;
	float accept_range_distance;

	bool yaw_reached;
	bool pos_reached;

	bool idle;										///< indicates if the system is following the waypoints or is waiting

	uint8_t current_partner_sysid;
	uint8_t current_partner_compid;
} mavlink_waypoint_storage_t;

mavlink_waypoint_storage_t waypoint_data;
pthread_mutex_t waypoint_mutex;

void *waypoint_loop (void *ptr);

void init_waypoint(void);


//mavlink_waypoint_t waypoints[MAVLINK_WPM_MAX_WP_COUNT];      ///< Currently active waypoints
int set_waypoint(mavlink_waypoint_t waypoint);
mavlink_waypoint_t *get_waypoint(uint16_t seq, mavlink_waypoint_t *waypoint);


//size
void set_waypoint_size(uint16_t size);
uint16_t get_waypoint_size(void);

//max_size
uint16_t get_waypoint_max_size(void);

//rcv_size
void set_waypoint_rcv_size(uint16_t size);
uint16_t get_waypoint_rcv_size(void);

//current_state
void set_waypoint_current_state(enum MAVLINK_WPM_STATES state);
enum MAVLINK_WPM_STATES get_waypoint_current_state(void);

//current_wp_id
int set_waypoint_current_wp_id(uint16_t wp_id);
uint16_t get_waypoint_current_wp_id(void);

//current_active_wp_id
int set_waypoint_current_active_wp_id(uint16_t wp_id);
uint16_t get_waypoint_current_active_wp_id(void);

//current_count
void set_waypoint_current_count(uint16_t size);
uint16_t get_waypoint_current_count(void);

//timestamp_lastaction
void set_waypoint_timestamp_lastaction(void);
uint64_t get_waypoint_timestamp_lastaction(void);

//timestamp_last_send_setpoint
void set_waypoint_timestamp_last_send_setpoint(void);
uint64_t get_waypoint_timestamp_last_send_setpoint(void);

//timestamp_lastoutside_orbit
void set_waypoint_timestamp_lastoutside_orbit(void);
uint64_t get_waypoint_timestamp_lastoutside_orbit(void);

//timestamp_firstinside_orbit
void set_waypoint_timestamp_firstinside_orbit(int reset);
uint64_t get_waypoint_timestamp_firstinside_orbit(void);

//timeout
void set_waypoint_timeout(uint64_t timeout);
uint64_t get_waypoint_timeout(void);

//delay_setpoint
void set_waypoint_delay_setpoint(uint32_t delay);
uint32_t get_waypoint_delay_setpoint(void);

//idle
bool get_waypoint_idle(void);

//accept_range_yaw;

//accept_range_distance;

//yaw_reached
void set_waypoint_yaw_reached(bool size);
bool get_waypoint_yaw_reached(void);

//pos_reached
void set_waypoint_pos_reached(bool size);
bool get_waypoint_pos_reached(void);

void set_waypoint_waypoints_current(uint16_t wp_id, bool value);
bool get_waypoint_waypoints_current(uint16_t wp_id);

void set_waypoint_current_partner_sysid(uint8_t sysid);
uint8_t get_waypoint_current_partner_sysid(void);

void set_waypoint_current_partner_compid(uint8_t compid);
uint8_t get_waypoint_current_partner_compid(void);

//////////////////////////////

void update_active_waypoint(uint16_t id);
float distance_to_point (float latA, float lonA, float altA, float latB, float lonB, float altB);
float waypoint_distance_to_point (uint16_t id, float lat, float lon, float alt);
float waypoint_distance_to_segment (uint16_t id, float lat, float lon, float alt);

//////////////////////////////

void send_mav_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t result);
void send_mav_waypoint_current(uint16_t seq);
void send_mav_waypoint_setpoint(uint8_t sysid, uint8_t compid, uint16_t seq);
void send_mav_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count);
void send_mav_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq);
void send_mav_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq);
void send_mav_waypoint_reached(uint16_t seq);












#endif /* WAYPOINT_H_ */
