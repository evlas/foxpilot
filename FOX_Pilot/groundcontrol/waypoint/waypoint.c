/*
 * waypoint.c
 *
 *  Created on: 01/set/2011
 *      Author: vito
 */
#include <stdio.h>
#include "../proto.h"

#include "waypoint.h"

mavlink_waypoint_storage_t waypoint_data;
pthread_mutex_t waypoint_mutex;

void *waypoint_loop (void *ptr) {
	int i;

	init_waypoint();

	while(true){
		//check for timed-out operations
		uint64_t now = microsSinceEpoch();

		if (((now - get_waypoint_timestamp_lastaction()) > get_waypoint_timeout())
				&& (get_waypoint_current_state() != MAVLINK_WPM_STATE_IDLE)) {
			set_waypoint_current_state(MAVLINK_WPM_STATE_IDLE);
			set_waypoint_current_count(0);
			set_waypoint_current_wp_id(-1);

			if(get_waypoint_size() == 0) {
				set_waypoint_current_active_wp_id(-1);
			}
		}

		//		if(((now - get_waypoint_timestamp_last_send_setpoint()) > get_waypoint_delay_setpoint())
		//				&& (get_waypoint_current_active_wp_id() < get_waypoint_size())) {
		//			send_mav_waypoint_setpoint(sysid, compid, get_waypoint_current_active_wp_id());
		//		}

		//check if the current waypoint was reached
		if (get_waypoint_pos_reached() && (!get_waypoint_idle())) {
			if (get_waypoint_current_active_wp_id() < get_waypoint_size()) {
				mavlink_waypoint_t cur_wp = get_waypoint(get_waypoint_current_active_wp_id());

				if (get_waypoint_timestamp_firstinside_orbit() == 0) {
					// Announce that last waypoint was reached

					send_mav_waypoint_reached(cur_wp.seq);
					set_waypoint_timestamp_firstinside_orbit(1);
				}

				// check if the MAV was long enough inside the waypoint orbit
				//if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))
				if((now - get_waypoint_timestamp_firstinside_orbit()) >= (cur_wp.param2 * 1000)) {
					if (cur_wp.autocontinue) {
						cur_wp.current = 0;
						if ((get_waypoint_current_active_wp_id() == (get_waypoint_size() - 1))
								&& (get_waypoint_size() > 1)) {
							//the last waypoint was reached, if auto continue is
							//activated restart the waypoint list from the beginning
							set_waypoint_current_active_wp_id(1);
						} else {
							if ((uint16_t)(get_waypoint_current_active_wp_id() + 1) < get_waypoint_size())
								set_waypoint_current_active_wp_id(get_waypoint_current_active_wp_id()+1);
						}

						// Fly to next waypoint
						set_waypoint_timestamp_firstinside_orbit(0);
						send_mav_waypoint_current(get_waypoint_current_active_wp_id());
						//send_mav_waypoint_setpoint(sysid, compid, get_waypoint_current_active_wp_id());

						set_waypoint_waypoints_current(get_waypoint_current_active_wp_id(), true);

						set_waypoint_pos_reached(false);
						set_waypoint_yaw_reached(false);
					}
				}
			}
		} else {
			set_waypoint_timestamp_lastoutside_orbit();
		}

		for(i=0;i<waypoint_data.size;i++){
			printf("waypoint_data.waypoints[%d].seq     %d\n",i,waypoint_data.waypoints[i].seq);
			printf("waypoint_data.waypoints[%d].current %d\n",i,waypoint_data.waypoints[i].current);
			printf("waypoint_data.waypoints[%d].command %d\n",i,waypoint_data.waypoints[i].command);
		}
		sleep(1);
	}
}

void init_waypoint(void) {
	// Set all waypoints to zero
	pthread_mutex_lock(&waypoint_mutex);
	// Set count to zero
	waypoint_data.current_count = 0;
	waypoint_data.size = 0;
	waypoint_data.max_size = MAVLINK_WPM_MAX_WP_COUNT;
	waypoint_data.current_state = MAVLINK_WPM_STATE_IDLE;
	waypoint_data.timestamp_lastaction = 0;
	waypoint_data.timestamp_last_send_setpoint = 0;
	waypoint_data.timeout = MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT;
	waypoint_data.delay_setpoint = MAVLINK_WPM_SETPOINT_DELAY_DEFAULT;
	waypoint_data.idle = false;      						///< indicates if the system is following the waypoints or is waiting
	waypoint_data.current_active_wp_id = -1;				///< id of current waypoint
	waypoint_data.yaw_reached = false;						///< boolean for yaw attitude reached
	waypoint_data.pos_reached = false;						///< boolean for position reached
	waypoint_data.timestamp_lastoutside_orbit = 0;			///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
	waypoint_data.timestamp_firstinside_orbit = 0;			///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value
	pthread_mutex_unlock(&waypoint_mutex);
}

//mavlink_waypoint_t waypoints[MAVLINK_WPM_MAX_WP_COUNT];      ///< Currently active waypoints
int set_waypoint(mavlink_waypoint_t wp) {
	int res=0;
	pthread_mutex_lock(&waypoint_mutex);
		waypoint_data.waypoints[wp.seq] = wp;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}
mavlink_waypoint_t get_waypoint(uint16_t seq) {
	mavlink_waypoint_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.waypoints[seq];
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//current_count
void set_waypoint_current_count(uint16_t size) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.current_count = size;
	pthread_mutex_unlock(&waypoint_mutex);
}
uint16_t get_waypoint_current_count(void) {
	uint16_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.current_count;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//size
void set_waypoint_size(uint16_t size) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.size = size;
	pthread_mutex_unlock(&waypoint_mutex);
}
uint16_t get_waypoint_size(void) {
	uint16_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.size;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//max_size
uint16_t get_waypoint_max_size(void) {
	uint16_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.max_size;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//current_state
void set_waypoint_current_state(enum MAVLINK_WPM_STATES state) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.current_state = state;
	pthread_mutex_unlock(&waypoint_mutex);
}
enum MAVLINK_WPM_STATES get_waypoint_current_state(void) {
	enum MAVLINK_WPM_STATES res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.current_state;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//timestamp_lastaction
void set_waypoint_timestamp_lastaction(void) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.timestamp_lastaction = microsSinceEpoch();
	pthread_mutex_unlock(&waypoint_mutex);
}
uint64_t get_waypoint_timestamp_lastaction(void) {
	uint64_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.timestamp_lastaction;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//timestamp_last_send_setpoint
void set_waypoint_timestamp_last_send_setpoint(void) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.timestamp_last_send_setpoint = microsSinceEpoch();
	pthread_mutex_unlock(&waypoint_mutex);
}

//timeout
void set_waypoint_timeout(uint64_t timeout) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.timeout = microsSinceEpoch() + timeout;
	pthread_mutex_unlock(&waypoint_mutex);
}
uint64_t get_waypoint_timeout(void) {
	uint64_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.timeout;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//delay_setpoint

//idle
bool get_waypoint_idle(void) {
	bool res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.idle;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//current_active_wp_id
int set_waypoint_current_wp_id(uint16_t wp_id) {
	int res = 0;

	if (wp_id<get_waypoint_max_size()) {
		pthread_mutex_lock(&waypoint_mutex);
		waypoint_data.current_wp_id = wp_id;
		pthread_mutex_unlock(&waypoint_mutex);
	} else {
		res = 1;
	}
	return(res);
}
uint16_t get_waypoint_current_wp_id(void) {
	uint16_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.current_wp_id;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//current_active_wp_id
int set_waypoint_current_active_wp_id(uint16_t wp_id) {
	int res = 0;

	if (wp_id<get_waypoint_max_size()) {
		pthread_mutex_lock(&waypoint_mutex);
		waypoint_data.current_active_wp_id = wp_id;
		pthread_mutex_unlock(&waypoint_mutex);
	} else {
		res = 1;
	}
	return(res);
}
uint16_t get_waypoint_current_active_wp_id(void) {
	uint16_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.current_active_wp_id;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//yaw_reached
void set_waypoint_yaw_reached(bool size) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.yaw_reached = size;
	pthread_mutex_unlock(&waypoint_mutex);
}

//pos_reached
void set_waypoint_pos_reached(bool size) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.pos_reached = size;
	pthread_mutex_unlock(&waypoint_mutex);
}
bool get_waypoint_pos_reached(void) {
	bool res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.pos_reached;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//timestamp_lastoutside_orbit
void set_waypoint_timestamp_lastoutside_orbit(void) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.timestamp_lastoutside_orbit = microsSinceEpoch();
	pthread_mutex_unlock(&waypoint_mutex);
}
uint64_t get_waypoint_timestamp_lastoutside_orbit(void) {
	uint64_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.timestamp_lastoutside_orbit;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//timestamp_firstinside_orbit
void set_waypoint_timestamp_firstinside_orbit(int reset) {
	pthread_mutex_lock(&waypoint_mutex);
	if(reset == 0) {
		waypoint_data.timestamp_firstinside_orbit = 0;
	} else {
		waypoint_data.timestamp_firstinside_orbit = microsSinceEpoch();
	}
	pthread_mutex_unlock(&waypoint_mutex);
}
uint64_t get_waypoint_timestamp_firstinside_orbit(void) {
	uint64_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.timestamp_firstinside_orbit;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

//rcv_size
void set_waypoint_rcv_size(uint16_t size) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.rcv_size = size;
	pthread_mutex_unlock(&waypoint_mutex);
}
uint16_t get_waypoint_rcv_size(void) {
	uint16_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.rcv_size;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}

void set_waypoint_waypoints_current(uint16_t wp_id, bool value) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.waypoints[wp_id].current = (uint8_t)value;
	pthread_mutex_unlock(&waypoint_mutex);
}
bool get_waypoint_waypoints_current(uint16_t wp_id) {
	bool res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.waypoints[wp_id].current;
	pthread_mutex_unlock(&waypoint_mutex);
	return(res);
}


///////////////////////////////////////////////////

// Sends an waypoint ack message
void send_mav_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t result) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_waypoint_ack_pack(get_param_value(PARAM_SYSTEM_ID),
			MAV_COMP_ID_WAYPOINTPLANNER,
			&msg,
			sysid,
			compid,
			result);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	send_udpserver(buf, len);
}

// Broadcasts the new target waypoint and directs the MAV to fly there
//
//  This function broadcasts its new active waypoint sequence number and
//  sends a message to the controller, advising it to fly to the coordinates
//  of the waypoint with a given orientation
//
//  @param seq The waypoint sequence number the MAV should fly to.

void send_mav_waypoint_current(uint16_t seq) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	if (seq < get_waypoint_size()) {

		mavlink_msg_waypoint_current_pack(get_param_value(PARAM_SYSTEM_ID),
				MAV_COMP_ID_WAYPOINTPLANNER,
				&msg,
				seq);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		send_udpserver(buf, len);
	}
}

//  @brief Directs the MAV to fly to a position
//
//  Sends a message to the controller, advising it to fly to the coordinates
//  of the waypoint with a given orientation
//
//  @param seq The waypoint sequence number the MAV should fly to.
void send_mav_waypoint_setpoint(uint8_t sysid, uint8_t compid, uint16_t seq) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	mavlink_waypoint_t waypoint;

	if (seq < get_waypoint_size()) {
		mavlink_local_position_setpoint_set_t position_control_set_point;

		// Send new NED or ENU setpoint to onbaord autopilot
		if (waypoint.frame == MAV_FRAME_GLOBAL) {

			mavlink_msg_local_position_setpoint_set_pack(get_param_value(PARAM_SYSTEM_ID),
					MAV_COMP_ID_WAYPOINTPLANNER,
					&msg,
					sysid,
					compid,
					waypoint.x,
					waypoint.y,
					waypoint.z,
					waypoint.param4);

			len = mavlink_msg_to_send_buffer(buf, &msg);

			send_udpserver(buf, len);
		}

		set_waypoint_timestamp_last_send_setpoint();
	}
}

void send_mav_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_waypoint_count_pack(get_param_value(PARAM_SYSTEM_ID),
			MAV_COMP_ID_WAYPOINTPLANNER,
			&msg,
			sysid,
			compid,
			count);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	send_udpserver(buf, len);
}

void send_mav_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	mavlink_waypoint_t waypoint;

	if (seq < get_waypoint_size()) {
		waypoint = get_waypoint(seq);

		mavlink_msg_waypoint_pack(get_param_value(PARAM_SYSTEM_ID),
				MAV_COMP_ID_WAYPOINTPLANNER,
				&msg,
				sysid,
				compid,
				waypoint.seq,
				waypoint.frame,
				waypoint.command,
				waypoint.current,
				waypoint.autocontinue,
				waypoint.param1,
				waypoint.param2,
				waypoint.param3,
				waypoint.param4,
				waypoint.x,
				waypoint.y,
				waypoint.z);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		send_udpserver(buf, len);
	}
}

void send_mav_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	if (seq < get_waypoint_max_size()) {
		//mavlink_msg_waypoint_request_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wpr);
		mavlink_msg_waypoint_request_pack(get_param_value(PARAM_SYSTEM_ID),
				MAV_COMP_ID_WAYPOINTPLANNER,
				&msg,
				sysid,
				compid,
				seq);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		send_udpserver(buf, len);
	}
}

//  @brief emits a message that a waypoint reached
//
//  This function broadcasts a message that a waypoint is reached.
//
//  @param seq The waypoint sequence number the MAV has reached.
void send_mav_waypoint_reached(uint16_t seq) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	if (seq < get_waypoint_max_size()) {
		//mavlink_msg_waypoint_request_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wpr);
		mavlink_msg_waypoint_reached_pack(get_param_value(PARAM_SYSTEM_ID),
				MAV_COMP_ID_WAYPOINTPLANNER,
				&msg,
				seq);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		send_udpserver(buf, len);
	}
}

/*
float mavlink_wpm_distance_to_point(uint16_t seq, float x, float y, float z) {
    if (seq < wpm.size) {
        mavlink_waypoint_t *cur = waypoints->at(seq);

        const float_vect3 A(cur->x, cur->y, cur->z);
        const float_vect3 C(x, y, z);

        return (C-A).length();
    }
    return -1.f;
}
*/
