/*
 * proto.c
 *
 *  Created on: 03/ago/2011
 *      Author: vito
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink.h>

#include <defines.h>
#include <config.h>

#include <sensori.h>
#include <attuatori.h>
#include <groundcontrol.h>
#include <pilota.h>

#include <math/range.h>

#include "proto.h"
#include "param.h"
#include "sys_state.h"
#include "radio.h"

#include "waypoint/waypoint.h"

gps_t gps_g_old;
uint64_t last_gps_g_old;
imu_t imu_g_old;
uint64_t last_imu_g_old;

void *protocol_loop(void *ptr) {
	FILE *fp;
	int i1, i2, i3, i4, i;
	uint64_t t1, t2, count = 0;
	long delta = 0, loop_d = 0;
	groundcontrol_t groundcontrol_g;
	init_protocol();

	while(1) {
		//50Hz Cycle
		t1 = microsSinceEpoch();

		read_groundcontrol(&groundcontrol_g);

		if ((microsSinceEpoch()-last_gps_g_old)>1000000) {
			read_gps_sensori(&gps_g_old);
			last_gps_g_old = microsSinceEpoch();
		}

		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 2 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (!(count % 25)) {
			// Send system state, mode, battery voltage, etc.
			send_system_state();

			// Send current onboard time
			send_mav_system_time();

			send_mav_system_time_utc();

			send_mav_vfr_hud();
		}

		// RAW SENSOR DATA
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_RAW_SENSORS].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_RAW_SENSORS].rate))) {
			//printf("RAW SENSOR DATA\n");
			send_mav_attitude();
		}
		// EXTENDED SYSTEM STATUS
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_EXTENDED_STATUS].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_EXTENDED_STATUS].rate))) {
			//printf("EXTENDED SYSTEM STATUS\n");
			send_mav_control_status();
		}
		// REMOTE CONTROL CHANNELS
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_RC_CHANNELS].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_RC_CHANNELS].rate))) {
			//printf("REMOTE CONTROL CHANNELS\n");
			send_mav_rc_channels_raw();
			send_mav_rc_channels_scaled();
			send_mav_servo_output_raw();
		}
		// RAW CONTROLLER
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_RAW_CONTROLLER].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_RAW_CONTROLLER].rate))) {
			//printf("RAW CONTROLLER\n");
		}
		// POSITION
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_POSITION].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_POSITION].rate))) {
			//printf("POSITION\n");
			send_mav_global_position_int();
		}
		// EXTRA1
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_EXTRA1].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_EXTRA1].rate))) {
			//printf("EXTRA1\n");
		}
		// EXTRA2
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_EXTRA2].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_EXTRA2].rate))) {
			//printf("EXTRA2\n");
		}
		// EXTRA3
		if ((groundcontrol_g.datastream[MAV_DATA_STREAM_EXTRA3].enable == 1) && !(count % (50/groundcontrol_g.datastream[MAV_DATA_STREAM_EXTRA3].rate))) {
			//printf("EXTRA3\n");
		}

		//		if(count < 49) {
		count++;
		//		} else {
		//			count = 0;
		//		}

		t2 = microsSinceEpoch();
		delta = t2 - t1;

		//		printf("delta %ld\n",delta);

		loop_d = 20000 - delta;

		if (loop_d > 0) {
			//attendo (1/50 sec - tempo impiegato fino a qui) microsec
			usleep(loop_d);
		}
	}
}

void init_protocol() {
	set_datastream(MAV_DATA_STREAM_ALL, 0, 0);
	set_datastream(MAV_DATA_STREAM_RAW_SENSORS, 1, 1);
	set_datastream(MAV_DATA_STREAM_EXTENDED_STATUS, 0, 0);
	set_datastream(MAV_DATA_STREAM_RC_CHANNELS, 0, 0);
	set_datastream(MAV_DATA_STREAM_RAW_CONTROLLER, 0, 0);
	set_datastream(MAV_DATA_STREAM_POSITION, 1, 1);
	set_datastream(MAV_DATA_STREAM_EXTRA1, 0, 0);
	set_datastream(MAV_DATA_STREAM_EXTRA2, 0, 0);
	set_datastream(MAV_DATA_STREAM_EXTRA3, 0, 0);
}

/////////////////////////////////////////////////////////////////////////////////////
//RECEIVE

int handle_message(uint8_t *buf, int dim) {
	int i = 0;
	unsigned int temp = 0;

	if (dim > 0) {
		// Something received - print out all bytes and parse packet
		mavlink_message_t msg;
		mavlink_status_t status;//mavlink_msg_system_time

		for (i = 0; i < dim; ++i) {
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {

			}
		}

		set_drop_rate(status.packet_rx_drop_count, status.packet_rx_success_count);

		printf("msg.msgid %d\n",msg.msgid);

		set_waypoint_current_partner_sysid(msg.sysid);
		set_waypoint_current_partner_compid(msg.compid);

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT: {
			//			mavlink_heartbeat_t heartbeat;

			//			mavlink_msg_heartbeat_decode(&msg, &heartbeat);
			//Vito che me ne faccio?
			//			send_mav_heartbeat();
		}
		break;
		case MAVLINK_MSG_ID_SYSTEM_TIME: {
			mavlink_system_time_t system_time;

			mavlink_msg_system_time_decode(&msg, &system_time);

			//Vito da verificare se c'è da filtrare
			if ((mavlink_msg_action_get_target(&msg) == get_param_value(PARAM_SYSTEM_ID)))  {
				//&& (mavlink_msg_action_get_target_component(&msg) == get_param_value(PARAM_COMPONENT_ID))) {
				if (!sys_time_clock_get_unix_offset()) {
					int64_t offset = ((int64_t) mavlink_msg_system_time_get_time_usec(&msg)) - (int64_t) microsSinceEpoch();
					sys_time_clock_set_unix_offset(offset);

					send_mav_statustext(0,"UNIX offset updated", 19);
				} else {
					perror("UNIX offset REFUSED");
					send_mav_statustext(0,"UNIX offset REFUSED", 19);
				}
			}
			send_mav_system_time_utc();
		}
		break;
		case MAVLINK_MSG_ID_PING: {
			mavlink_ping_t ping;

			mavlink_msg_ping_decode(&msg, &ping);

			if (ping.target_system == 0) {
				//&& ping.target_component == 0) {
				send_mav_ping(msg.sysid, msg.compid, ping.seq);
			}
		}
		break;
		case MAVLINK_MSG_ID_ACTION: {
			mavlink_action_t action;

			mavlink_msg_action_decode(&msg, &action);
			if ((action.target == get_param_value(PARAM_SYSTEM_ID))) {
				//				&& (action.target_component == get_param_value(PARAM_COMPONENT_ID))) {
				//mavlink_msg_action_get_target_component(&msg);

				//printf("TARGET %d ACTION %d\n",action.target, mavlink_msg_action_get_action(&msg));
				handle_action(msg);
			}
		}
		break;
		case MAVLINK_MSG_ID_SET_MODE: {
			mavlink_set_mode_t mode;

			mavlink_msg_set_mode_decode(&msg, &mode);

			// Check if this system should change the mode
			if (mode.target == get_param_value(PARAM_SYSTEM_ID)) {
				set_sys_state_mode(mode.mode);

				//update_system_statemachine();

				// Emit current mode
				send_mav_status();
			}
		}
		break;
		case MAVLINK_MSG_ID_SET_NAV_MODE: {
			mavlink_set_nav_mode_t nav_mode;

			mavlink_msg_set_nav_mode_decode(&msg, &nav_mode);

			if (nav_mode.target == get_param_value(PARAM_SYSTEM_ID)) {
				set_sys_state_nav_mode(nav_mode.nav_mode);

				// Emit current mode
				send_mav_status();
			}
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			mavlink_param_request_read_t set;

			mavlink_msg_param_request_read_decode(&msg, &set);

			// Check if this message is for this system
			if (((uint8_t) set.target_system == get_param_value(PARAM_SYSTEM_ID))) {
				//&& ((uint8_t) set.target_component == get_param_value(PARAM_COMPONENT_ID))) {

				if (set.param_id[0] == '\0') {
					// Choose parameter based on index
					if (set.param_index < ONBOARD_PARAM_COUNT) {
						// Report back value
						send_mav_param_value(set.param_index);
					}
				} else {
					int i;
					i=get_param_id_with_name(set.param_id);
					// Check if matched
					if (i > -1) {
						// Report back value
						send_mav_param_value(i);
					}
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			mavlink_param_request_list_t list;

			mavlink_msg_param_request_list_decode(&msg, &list);
			if (((uint8_t) list.target_system == get_param_value(PARAM_SYSTEM_ID))) {
				//					&& ((uint8_t) list.target_component == get_param_value_groundcontrol(PARAM_COMPONENT_ID))) {
				for (i=0;i<ONBOARD_PARAM_COUNT;i++) {
					send_mav_param_value(i);
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET: {
			int i;
			mavlink_param_set_t set;

			mavlink_msg_param_set_decode(&msg, &set);

			// Check if this message is for this system
			if (((uint8_t) set.target_system == get_param_value(PARAM_SYSTEM_ID))) {
				// && ((uint8_t) set.target_component == get_param_value(PARAM_COMPONENT_ID))) {
				i = set_param_value_with_name(set.param_id, set.param_value);

				// Report back new value
				if (i > -1) {
					handle_param(i);
					send_mav_param_value(i);
				}
			}
		}
		break;


		case MAVLINK_MSG_ID_LOCAL_POSITION: {
            if(get_waypoint_current_active_wp_id() < get_waypoint_size()) {
                mavlink_waypoint_t wp;
                get_waypoint(get_waypoint_current_active_wp_id(), &wp);

                if(wp.frame == MAV_FRAME_LOCAL_ENU) {
                    mavlink_local_position_t pos;
                    float orbit, dist;
                    mavlink_msg_local_position_decode(&msg, &pos);
                    //if (debug) printf("Received new position: x: %f | y: %f | z: %f\n", pos.x, pos.y, pos.z);

                    set_waypoint_pos_reached(false);

                    // compare current position (given in message) with current waypoint
                    orbit = wp.param1;

                    if (wp.param2 == 0) {
						// FIXME segment distance
                        dist = waypoint_distance_to_segment(get_waypoint_current_active_wp_id(), pos.x, pos.y, pos.z);
                    } else {
                        dist = waypoint_distance_to_point(get_waypoint_current_active_wp_id(), pos.x, pos.y, pos.z);
                    }

                    if ((dist >= 0.f) && (dist <= orbit) && (get_waypoint_yaw_reached())) {
                    	set_waypoint_pos_reached(true);
                    }
                }
            }

        }
		break;



		case MAVLINK_MSG_ID_WAYPOINT: {
			mavlink_waypoint_t wp;
			mavlink_msg_waypoint_decode(&msg, &wp);

			if(wp.target_system == get_param_value(PARAM_SYSTEM_ID)) {
				set_waypoint_timestamp_lastaction();

				//ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
				if (((get_waypoint_current_state() == MAVLINK_WPM_STATE_GETLIST) && (wp.seq == 0))
						|| ((get_waypoint_current_state() == MAVLINK_WPM_STATE_GETLIST_GETWPS)
								&& ((wp.seq == get_waypoint_current_wp_id())
										&& (wp.seq < get_waypoint_current_count())))) {

					set_waypoint_current_state(MAVLINK_WPM_STATE_GETLIST_GETWPS);

					set_waypoint(wp);

					set_waypoint_current_wp_id(wp.seq + 1);

					if((get_waypoint_current_wp_id() == get_waypoint_current_count())
							&& (get_waypoint_current_state() == MAVLINK_WPM_STATE_GETLIST_GETWPS)) {
						int i;
						send_mav_waypoint_ack(msg.sysid, msg.compid, 0);

						//						if (get_waypoint_current_active_wp_id() > (get_waypoint_rcv_size()-1)) {
						//							set_waypoint_current_active_wp_id(get_waypoint_rcv_size()-1);
						//						}

						// switch the waypoints list
						// FIXME CHECK!!!
						//for (i = 0; i < get_waypoint_current_count(); ++i) {
						//	wpm.waypoints[i] = wpm.rcv_waypoints[i];
						//}
						set_waypoint_size(get_waypoint_current_count());

						//get the new current waypoint
						for(i = 0; i < get_waypoint_size(); i++) {
							if (get_waypoint_waypoints_current(i) == 1) {
								set_waypoint_current_active_wp_id(i);

								set_waypoint_yaw_reached(false);
								set_waypoint_pos_reached(false);
								send_mav_waypoint_current(get_waypoint_current_active_wp_id());
								send_mav_waypoint_setpoint(msg.sysid, msg.compid, i);
								//wpm.timestamp_firstinside_orbit = 0;
								set_waypoint_timestamp_firstinside_orbit(0);
								break;
							}
						}

						if (i == get_waypoint_size()) {
							set_waypoint_current_active_wp_id(-1);
							set_waypoint_yaw_reached(false);
							set_waypoint_pos_reached(false);
							//wpm.timestamp_firstinside_orbit = 0;
							set_waypoint_timestamp_firstinside_orbit(0);
						}

						set_waypoint_current_state(MAVLINK_WPM_STATE_IDLE);
					} else {
						send_mav_waypoint_request(msg.sysid, msg.compid, get_waypoint_current_wp_id());
					}
				} else {
					if (get_waypoint_current_state() == MAVLINK_WPM_STATE_IDLE) {
						//we're done receiving waypoints, answer with ack.
						send_mav_waypoint_ack(msg.sysid, msg.compid, 0);
					}
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_WAYPOINT_REQUEST: {
			mavlink_waypoint_request_t wpr;

			mavlink_msg_waypoint_request_decode(&msg, &wpr);

			if(wpr.target_system == get_param_value(PARAM_SYSTEM_ID)) {
				set_waypoint_timestamp_lastaction();

				//ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
				if (((get_waypoint_current_state() == MAVLINK_WPM_STATE_SENDLIST)
						&& (wpr.seq == 0))
						|| ((get_waypoint_current_state() == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)
								&& ((wpr.seq == get_waypoint_current_wp_id())
										|| (wpr.seq == (get_waypoint_current_wp_id() + 1)))
										&& (wpr.seq < get_waypoint_size()))) {
					printf("ci passo\n");

					set_waypoint_current_state(MAVLINK_WPM_STATE_SENDLIST_SENDWPS);
					set_waypoint_current_wp_id(wpr.seq);
					send_mav_waypoint(msg.sysid, msg.compid, wpr.seq);
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT: {
			mavlink_waypoint_set_current_t wpsc;

			mavlink_msg_waypoint_set_current_decode(&msg, &wpsc);

			if(wpsc.target_system == get_param_value(PARAM_SYSTEM_ID)) {
				set_waypoint_timestamp_lastaction();

				if (get_waypoint_current_state() == MAVLINK_WPM_STATE_IDLE) {
					if (wpsc.seq < get_waypoint_size()) {
						int i;
						set_waypoint_current_active_wp_id(wpsc.seq);

						for(i = 0; i < get_waypoint_size(); i++) {
							if (i == get_waypoint_current_active_wp_id()) {
								set_waypoint_waypoints_current(i, true);
							} else {
								set_waypoint_waypoints_current(i, false);
							}
						}
						set_waypoint_yaw_reached(false);
						set_waypoint_pos_reached(false);
						send_mav_waypoint_current(get_waypoint_current_active_wp_id());
						send_mav_waypoint_setpoint(msg.sysid, msg.compid, get_waypoint_current_active_wp_id());
						set_waypoint_timestamp_firstinside_orbit(0);
					}
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST: {
			mavlink_waypoint_request_list_t wprl;

			mavlink_msg_waypoint_request_list_decode(&msg, &wprl);

			if(wprl.target_system == get_param_value(PARAM_SYSTEM_ID)) {
				set_waypoint_timestamp_lastaction();


				if ((get_waypoint_current_state() == MAVLINK_WPM_STATE_IDLE)
						|| (get_waypoint_current_state() == MAVLINK_WPM_STATE_SENDLIST)){
					if (get_waypoint_size() > 0) {
						set_waypoint_current_state(MAVLINK_WPM_STATE_SENDLIST);
						set_waypoint_current_active_wp_id(0);
					}
					set_waypoint_current_count(get_waypoint_size());
					send_mav_waypoint_count(msg.sysid, msg.compid, get_waypoint_current_count());
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_WAYPOINT_COUNT: {
			mavlink_waypoint_count_t wpc;

			mavlink_msg_waypoint_count_decode(&msg, &wpc);

			if(wpc.target_system == get_param_value(PARAM_SYSTEM_ID)) {
				set_waypoint_timestamp_lastaction();

				if ((get_waypoint_current_state() == MAVLINK_WPM_STATE_IDLE)
						|| ((get_waypoint_current_state() == MAVLINK_WPM_STATE_GETLIST)
								&& (get_waypoint_current_active_wp_id() == 0))) {

					if (wpc.count > 0) {
						set_waypoint_current_state(MAVLINK_WPM_STATE_GETLIST);
						set_waypoint_current_active_wp_id(0);
						set_waypoint_current_count(wpc.count);

						set_waypoint_rcv_size(0);

						send_mav_waypoint_request(msg.sysid, msg.compid, get_waypoint_current_active_wp_id());

					} else if (wpc.count == 0) {
						set_waypoint_rcv_size(0);
						set_waypoint_current_active_wp_id(-1);
						set_waypoint_yaw_reached(false);
						set_waypoint_pos_reached(false);
					}
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL: {
			mavlink_waypoint_clear_all_t wpca;

			mavlink_msg_waypoint_clear_all_decode(&msg, &wpca);

			if((wpca.target_system == get_param_value(PARAM_SYSTEM_ID))
					&& (get_waypoint_current_state() == MAVLINK_WPM_STATE_IDLE)){
				set_waypoint_timestamp_lastaction();

				// Delete all waypoints
				set_waypoint_size(0);
				set_waypoint_current_active_wp_id(-1);
				set_waypoint_yaw_reached(false);
				set_waypoint_pos_reached(false);
			}
		}
		break;
		case MAVLINK_MSG_ID_WAYPOINT_ACK: {
			mavlink_waypoint_ack_t wpa;

			mavlink_msg_waypoint_ack_decode(&msg, &wpa);

			if(wpa.target_system == get_param_value(PARAM_SYSTEM_ID)) {

				set_waypoint_timestamp_lastaction();

				if ((get_waypoint_current_state() == MAVLINK_WPM_STATE_SENDLIST)
						|| (get_waypoint_current_state() == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)) {
					if (get_waypoint_current_wp_id() == (get_waypoint_size()-1)) {
						set_waypoint_current_state(MAVLINK_WPM_STATE_IDLE);
						set_waypoint_current_wp_id(0);
					}
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET: {
			mavlink_local_position_setpoint_set_t point;

			mavlink_msg_local_position_setpoint_set_decode(&msg, &point);
			//			mavlink_position_control_setpoint_set_t pos;

			if (((uint8_t) point.target_system == get_param_value(PARAM_SYSTEM_ID))) {
				//&& ((uint8_t) point.target_component == get_param_value(PARAM_COMPONENT_ID))) {
				/*
				if (global_data.param[PARAM_POSITIONSETPOINT_ACCEPT] == 1) {
					//			global_data.position_setpoint.x = pos.x;
					//			global_data.position_setpoint.y = pos.y;
					//			global_data.position_setpoint.z = pos.z;
					debug_message_buffer("Position setpoint updated. OLD?\n");
				} else {
					debug_message_buffer(
							"Position setpoint recieved but not updated. OLD?\n");
				}

				// Send back a message confirming the new position
				mavlink_msg_position_control_setpoint_send(MAVLINK_COMM_0, pos.id,
						pos.x, pos.y, pos.z, pos.yaw);
				mavlink_msg_position_control_setpoint_send(MAVLINK_COMM_1, pos.id,
						pos.x, pos.y, pos.z, pos.yaw);
			}
				 */
			}
		}
		break;
		case MAVLINK_MSG_ID_SET_ALTITUDE: {
			mavlink_set_altitude_t set;

			mavlink_msg_set_altitude_decode(&msg, &set);
			if ((set.target == (uint8_t) get_param_value(PARAM_SYSTEM_ID))) {
				//set_altitude(set.mode);
				printf("set altitude in : %d\n", set.mode);
			}
		}
		break;
		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
			mavlink_request_data_stream_t stream;

			mavlink_msg_request_data_stream_decode(&msg, &stream);

			if ((stream.target_system == (uint8_t) get_param_value(PARAM_SYSTEM_ID))) {
				//				&& ((uint8_t) stream.target_component == get_param_value(PARAM_COMPONENT_ID))) {
				switch (stream.req_stream_id) {
				case MAV_DATA_STREAM_ALL: // UNIMPLEMENTED
					set_datastream(MAV_DATA_STREAM_ALL, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_RAW_SENSORS: // RAW SENSOR DATA
					//					global_data.param[PARAM_SEND_SLOT_RAW_IMU] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_RAW_SENSORS, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_EXTENDED_STATUS: // EXTENDED SYSTEM STATUS
					//					global_data.param[PARAM_SEND_SLOT_ATTITUDE] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_EXTENDED_STATUS, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_RC_CHANNELS: // REMOTE CONTROL CHANNELS
					//					global_data.param[PARAM_SEND_SLOT_REMOTE_CONTROL] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_RC_CHANNELS, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_RAW_CONTROLLER: // RAW CONTROLLER
					//global_data.param[PARAM_SEND_SLOT_DEBUG_5] = stream.start_stop;
					//global_data.param[PARAM_SEND_SLOT_DEBUG_3] = stream.start_stop;
					//					global_data.param[PARAM_SEND_SLOT_CONTROLLER_OUTPUT] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_RAW_CONTROLLER, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_POSITION: // POSITION
					//					global_data.param[PARAM_SEND_SLOT_DEBUG_5] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_POSITION, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_EXTRA1: // EXTRA1
					//					global_data.param[PARAM_SEND_SLOT_DEBUG_2] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_EXTRA1, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_EXTRA2: // EXTRA2
					//					global_data.param[PARAM_SEND_SLOT_DEBUG_4] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_EXTRA2, stream.req_message_rate, stream.start_stop);
					break;
				case MAV_DATA_STREAM_EXTRA3: // EXTRA3
					//					global_data.param[PARAM_SEND_SLOT_DEBUG_6] = stream.start_stop;
					set_datastream(MAV_DATA_STREAM_EXTRA3, stream.req_message_rate, stream.start_stop);
					break;
				default:
					// Do nothing
					break;
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_MANUAL_CONTROL: {
			mavlink_manual_control_t manual_control;
			manual_ctrl_t remote;
			mavlink_msg_manual_control_decode(&msg, &manual_control);

			//Controllo manuale
			//completare
			if (((uint8_t) manual_control.target == get_param_value(PARAM_SYSTEM_ID))) {
				remote.roll = mavlink_msg_manual_control_get_roll(&msg);
				remote.roll_manual = mavlink_msg_manual_control_get_roll_manual(&msg);
				remote.pitch = mavlink_msg_manual_control_get_pitch(&msg);
				remote.pitch_manual = mavlink_msg_manual_control_get_pitch_manual(&msg);
				remote.yaw = mavlink_msg_manual_control_get_yaw(&msg);
				remote.yaw_manual = mavlink_msg_manual_control_get_yaw_manual(&msg);
				remote.thrust = mavlink_msg_manual_control_get_thrust(&msg);
				remote.thrust_manual = mavlink_msg_manual_control_get_thrust_manual(&msg);

				set_sys_state_manual_ctrl(remote);
			}


		}
		break;
		case MAVLINK_MSG_ID_COMMAND: {
			mavlink_command_t command;

			mavlink_msg_command_decode(&msg, &command);

			if (((uint8_t) command.target_system == get_param_value(PARAM_SYSTEM_ID))) {
				//&& ((uint8_t) command.target_component == get_param_value(PARAM_COMPONENT_ID))) {
				printf("MAVLINK_MSG_ID_COMMAND\n");
				send_mav_command_ack((float)mavlink_msg_command_get_command(&msg), 1.0);
			}
		}
		break;
		default:
			send_mav_statustext(0,"Messaggio Non Gestito", 21);
		}

	}
	memset(buf, 0, MAV_BUFFER_LENGTH);
}

int handle_action(mavlink_message_t msg) {
	int i = 0, ok_ko = 0;

	switch(mavlink_msg_action_get_action(&msg)) {
	case MAV_ACTION_HOLD:
		break;
	case MAV_ACTION_MOTORS_START:
		if ((get_sys_state_mode() > MAV_MODE_LOCKED) &&
				(get_sys_state_status() == MAV_STATE_STANDBY) &&
				(get_sys_state_nav_mode() == MAV_NAV_GROUNDED)) {
			if (set_sys_state_status(MAV_STATE_ACTIVE)) {
				if (get_sys_state_mode() == MAV_MODE_MANUAL) {
					set_sys_state_nav_mode(MAV_NAV_FREE_DRIFT);
				} else {
					set_sys_state_nav_mode(MAV_NAV_GROUNDED);
				}
				set_sys_state_fly(FLY_STARTING);
				ok_ko=1;
			} else {
				send_mav_statustext(0, "MOTORS_START Impossibile", 24);
			}
		}
		break;
	case MAV_ACTION_LAUNCH:
		if ((get_sys_state_mode() > MAV_MODE_LOCKED) &&
				(get_sys_state_status() == MAV_STATE_ACTIVE) &&
				(get_sys_state_nav_mode() == MAV_NAV_GROUNDED)) {
			if (set_sys_state_status(MAV_STATE_ACTIVE)) {
				set_sys_state_nav_mode(MAV_NAV_GROUNDED);
				set_sys_state_fly(FLY_STARTING);
				usleep(20000);
				set_sys_state_nav_mode(MAV_NAV_LIFTOFF);
				set_sys_state_fly(FLY_TAKE_OFF);
				ok_ko=1;
			} else {
				send_mav_statustext(0, "LAUNCH Impossibile", 18);
			}
		}
		break;
	case MAV_ACTION_RETURN:
		if ((get_sys_state_mode() > MAV_MODE_LOCKED) &&
				(get_sys_state_status() > MAV_STATE_CALIBRATING) &&
				(get_sys_state_nav_mode() > MAV_NAV_GROUNDED)) {
			if (set_sys_state_status(MAV_STATE_ACTIVE)) {
				set_sys_state_nav_mode(MAV_NAV_RETURNING);
				//setta waypoint home?
				//e poi setta landing?
				set_sys_state_fly(FLY_LANDING);
				ok_ko=1;
			} else {
				send_mav_statustext(0, "RETURN Impossibile", 18);
			}
		}
		break;
	case MAV_ACTION_EMCY_LAND:
		if ((get_sys_state_mode() > MAV_MODE_LOCKED) &&
				(get_sys_state_status() > MAV_STATE_CALIBRATING) &&
				(get_sys_state_nav_mode() > MAV_NAV_GROUNDED)) {
			if (set_sys_state_status(MAV_STATE_EMERGENCY)) {
				set_sys_state_nav_mode(MAV_NAV_LANDING);
				set_sys_state_fly(FLY_LANDING);

				ok_ko=1;
			} else {
				send_mav_statustext(0, "EMCY_LAND Impossibile", 21);
			}
		}
		break;
	case MAV_ACTION_EMCY_KILL:
		if ((get_sys_state_mode() == MAV_MODE_LOCKED) &&
				(get_sys_state_status() > MAV_STATE_CALIBRATING) &&
				(get_sys_state_nav_mode() == MAV_NAV_GROUNDED)) {
			if (set_sys_state_status(MAV_STATE_EMERGENCY)) {
				set_sys_state_nav_mode(MAV_NAV_GROUNDED);
				ok_ko=1;
			} else {
				send_mav_statustext(0, "EMCY_KILL Impossibile", 21);
			}
		}
		break;
	case MAV_ACTION_CONFIRM_KILL:
		if (get_sys_state_nav_mode() == MAV_NAV_GROUNDED) {
			if (set_sys_state_status(MAV_STATE_POWEROFF)) {
				ok_ko=1;
			} else {
				send_mav_statustext(0, "CONFIRM_KILL Impossibile", 24);
			}
		}
		break;
	case MAV_ACTION_CONTINUE:
		if (get_sys_state_nav_mode() == MAV_NAV_HOLD) {
			//hold position
			set_sys_state_nav_mode_prev();
			ok_ko=1;
		} else {
			send_mav_statustext(0, "CONTINUE Impossibile", 20);
		}
		break;
	case MAV_ACTION_MOTORS_STOP:
		if ((get_sys_state_mode() > MAV_MODE_LOCKED) &&
				(get_sys_state_status() > MAV_STATE_CALIBRATING) &&
				((get_sys_state_nav_mode() == MAV_NAV_GROUNDED) ||
						(get_sys_state_nav_mode() == MAV_NAV_FREE_DRIFT) ||
						(get_sys_state_nav_mode() == MAV_NAV_RETURNING))) {
			if (set_sys_state_status(MAV_STATE_STANDBY)) {
				set_sys_state_nav_mode(MAV_NAV_GROUNDED);
				set_sys_state_fly(FLY_IDLE);
				ok_ko=1;
			} else {
				send_mav_statustext(0, "MOTORS_STOP Impossibile", 23);
			}
		}
		break;
	case MAV_ACTION_HALT:
		if (get_sys_state_nav_mode() > MAV_NAV_GROUNDED) {
			//hold position
			set_sys_state_nav_mode(MAV_NAV_HOLD);
			ok_ko=1;
		} else {
			send_mav_statustext(0, "HALT Impossibile", 16);
		}
		break;
	case MAV_ACTION_SHUTDOWN:
		if (get_sys_state_nav_mode() == MAV_NAV_GROUNDED) {
			if (set_sys_state_status(MAV_STATE_POWEROFF)) {
				ok_ko=1;
			} else {
				send_mav_statustext(0, "SHUTDOWN Impossibile", 20);
			}
		}
		break;
	case MAV_ACTION_REBOOT:
		if (get_sys_state_nav_mode() == MAV_NAV_GROUNDED) {
			if (set_sys_state_status(MAV_STATE_POWEROFF)) {
				ok_ko=1;
			} else {
				send_mav_statustext(0, "REBOOT Impossibile", 18);
			}
		}
		break;
	case MAV_ACTION_SET_MANUAL:
		break;
	case MAV_ACTION_SET_AUTO:
		break;
	case MAV_ACTION_STORAGE_READ:
		param_read_all();
		for (i=0;i<ONBOARD_PARAM_COUNT;i++) {
			handle_param(i);
			send_mav_param_value(i);
		}
		if (get_sys_state_status() == MAV_STATE_BOOT) {
			set_sys_state_status(MAV_STATE_STANDBY);
		}
		ok_ko=1;
		break;
	case MAV_ACTION_STORAGE_WRITE:
		param_write_all();
		for (i=0;i<ONBOARD_PARAM_COUNT;i++) {
			handle_param(i);
			send_mav_param_value(i);
		}
		ok_ko=1;
		break;
	case MAV_ACTION_CALIBRATE_RC:
		//rc_calibration();
		//ok_ko=1;
		break;
	case MAV_ACTION_CALIBRATE_GYRO:
		//start_gyro_calibration();
		//ok_ko=1;
		break;
	case MAV_ACTION_CALIBRATE_MAG:
		//start_mag_calibration();
		//ok_ko=1;
		break;
	case MAV_ACTION_CALIBRATE_ACC:
		break;
	case MAV_ACTION_CALIBRATE_PRESSURE:
		//start_pressure_calibration();
		//ok_ko=1;
		break;
	case MAV_ACTION_REC_START:
		break;
	case MAV_ACTION_REC_PAUSE:
		break;
	case MAV_ACTION_REC_STOP:
		break;
	case MAV_ACTION_TAKEOFF:
		if ((get_sys_state_mode() > MAV_MODE_LOCKED) &&
				(get_sys_state_status() > MAV_STATE_CALIBRATING) &&
				(get_sys_state_nav_mode() == MAV_NAV_GROUNDED)) {
			if (set_sys_state_status(MAV_STATE_ACTIVE)) {
				set_sys_state_nav_mode(MAV_NAV_GROUNDED);
				set_sys_state_fly(FLY_STARTING);
				usleep(20000);
				set_sys_state_nav_mode(MAV_NAV_LIFTOFF);
				set_sys_state_fly(FLY_TAKE_OFF);
				ok_ko=1;
			} else {
				send_mav_statustext(0, "TAKEOFF Impossibile", 19);
			}
		}
		break;
	case MAV_ACTION_NAVIGATE:
		break;
	case MAV_ACTION_LAND:
		break;
	case MAV_ACTION_LOITER:
		if (get_sys_state_nav_mode() > MAV_NAV_GROUNDED) {
			//hold position
			set_sys_state_nav_mode(MAV_NAV_LOITER);
			ok_ko=1;
		} else {
			send_mav_statustext(0, "LOITER Impossibile", 18);
		}
		break;
	case MAV_ACTION_SET_ORIGIN:
		// If not flying
		if (!sys_state_is_flying()) {
			gps_set_local_origin();
			ok_ko=1;
		}
		break;
	case MAV_ACTION_RELAY_ON:
		break;
	case MAV_ACTION_RELAY_OFF:
		break;
	case MAV_ACTION_GET_IMAGE:
		break;
	case MAV_ACTION_VIDEO_START:
		break;
	case MAV_ACTION_VIDEO_STOP:
		break;
	case MAV_ACTION_RESET_MAP:
		break;
	case MAV_ACTION_RESET_PLAN:
		break;
	case MAV_ACTION_DELAY_BEFORE_COMMAND:
		break;
	case MAV_ACTION_ASCEND_AT_RATE:
		break;
	case MAV_ACTION_CHANGE_MODE:
		break;
	case MAV_ACTION_LOITER_MAX_TURNS:
		break;
	case MAV_ACTION_LOITER_MAX_TIME:
		break;
	default:
		printf("ACTION %d\n",mavlink_msg_action_get_action(&msg));
		break;
	}
	return(send_mav_action_ack(mavlink_msg_action_get_action(&msg), ok_ko));
}


/////////////////////////////////////////////////////////////////////////////////////
//SEND

void send_system_state(void) {
	// Send heartbeat to announce presence of this system
	send_mav_heartbeat();

	// Send system status
	send_mav_status();
}


/*Send Heartbeat */
/*
MAVLINK_MSG_ID_HEARTBEAT   0
 */
int send_mav_heartbeat(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_heartbeat_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			get_param_value(PARAM_SYSTEM_TYPE),
			get_param_value(PARAM_AUTOPILOT_TYPE));
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/*Send Version */
/*
MAVLINK_MSG_ID_BOOT   1
 */
int send_mav_boot(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_boot_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			get_param_value(PARAM_SW_VERSION));
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/*Send System Time*/
/*
MAVLINK_MSG_ID_SYSTEM_TIME   2
 */
int send_mav_system_time(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_system_time_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			microsSinceEpoch());
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/*Send Ping*/
/*
MAVLINK_MSG_ID_PING 3
 */
int send_mav_ping(uint8_t target_system, uint8_t target_component, uint32_t seq) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	//	msg.msgid;
	//	msg.compid

	mavlink_msg_ping_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			seq,
			target_system,
			target_component,
			microsSinceEpoch());
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/*Send System Time UTC*/
/*
MAVLINK_MSG_ID_SYSTEM_TIME_UTC 4
 */
int send_mav_system_time_utc(void) {
	int len;
	time_t rawtime;
	struct tm * timeinfo;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	uint32_t utc_date = timeinfo->tm_mday * 10000 + timeinfo->tm_mon * 100 + (timeinfo->tm_year+1900)-2000;
	uint32_t utc_time = timeinfo->tm_hour * 10000 + timeinfo->tm_min * 100 + timeinfo->tm_sec;

	mavlink_msg_system_time_utc_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			utc_date,
			utc_time);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/*Send System Time UTC*/
/*
MAVLINK_MSG_ID_PARAM_REQUEST_READ 20
 */
int send_mav_param_value(uint16_t param_id) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH], name[ONBOARD_PARAM_NAME_LENGTH];

	memset(name, 0, ONBOARD_PARAM_NAME_LENGTH);

	mavlink_msg_param_value_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			(int8_t*) get_param_name(name, param_id),
			get_param_value(param_id),
			ONBOARD_PARAM_COUNT,
			param_id);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send attitude */
/*
MAVLINK_MSG_ID_ATTITUDE   30
 */
int send_mav_attitude(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	imu_t imu_g;
	uint64_t last_imu_g;
	float vroll, vpitch, vyaw;

	read_imu_sensori(&imu_g);
	last_imu_g = microsSinceEpoch();

	if (imu_g.roll>imu_g_old.roll) {
		vroll = D2R*((imu_g.roll-imu_g_old.roll)/100.0)/(last_imu_g-last_imu_g_old);
	} else {
		vroll = -D2R*((imu_g.roll-imu_g_old.roll)/100.0)/(last_imu_g-last_imu_g_old);
	}
	if (imu_g.roll>imu_g_old.roll) {
		vpitch= D2R*((imu_g.pitch-imu_g_old.pitch)/100.0)/(last_imu_g-last_imu_g_old);
	}else {
		vpitch= -D2R*((imu_g.pitch-imu_g_old.pitch)/100.0)/(last_imu_g-last_imu_g_old);
	}

	if (imu_g.roll>imu_g_old.roll) {
		vyaw = D2R*((imu_g.yaw-imu_g_old.yaw)/100.0)/(last_imu_g-last_imu_g_old);
	}else {
		vyaw = -D2R*((imu_g.yaw-imu_g_old.yaw)/100.0)/(last_imu_g-last_imu_g_old);
	}

	mavlink_msg_attitude_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			microsSinceEpoch(),
			D2R*imu_g.roll/100.0,
			D2R*imu_g.pitch/100.0,
			D2R*imu_g.yaw/100.0,
			vroll,
			vpitch,
			vyaw);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Local Position */
/*
MAVLINK_MSG_ID_LOCAL_POSITION   31
 */
int send_mav_local_position(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	gps_t gps_g;
	uint64_t last_gps_g;
	float lat,lon;
	double V;

	read_gps_sensori(&gps_g);
	last_gps_g = microsSinceEpoch();

	lat = (gps_g.latitude - gps_g_old.latitude)/(last_gps_g-last_gps_g_old);
	lon = (gps_g.longitude - gps_g_old.longitude)/(last_gps_g-last_gps_g_old);
	if ((V = sqrt(pow(lat,2.0)+pow(lon,2.0)))==0) V=0.0000001;

	mavlink_msg_local_position_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			last_gps_g,
			gps_g.latitude/10000000.0,
			gps_g.longitude/10000000.0,
			gps_g.altitude/100.0,
			(gps_g.speed/100.0 * lat)/V,
			(gps_g.speed/100.0 * lon)/V,
			((gps_g.altitude - gps_g_old.altitude)/100.0)/((last_gps_g-last_gps_g_old)/1000000.0));

	len = mavlink_msg_to_send_buffer(buf, &msg);

	return(send_udpserver(buf, len));
}

/* Send Global Position */
/*
MAVLINK_MSG_ID_GLOBAL_POSITION   33
 */
int send_mav_global_position(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	gps_t gps_g;
	uint64_t last_gps_g;
	float lat,lon;
	double V;

	read_gps_sensori(&gps_g);
	last_gps_g = microsSinceEpoch();

	lat = (gps_g.latitude - gps_g_old.latitude)/(last_gps_g-last_gps_g_old);
	lon = (gps_g.longitude - gps_g_old.longitude)/(last_gps_g-last_gps_g_old);
	if ((V = sqrt(pow(lat,2.0)+pow(lon,2.0)))==0) V=0.0000001;

	mavlink_msg_global_position_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			last_gps_g,
			gps_g.latitude/10000000.0,
			gps_g.longitude/10000000.0,
			gps_g.altitude/100.0,
			(gps_g.speed/100.0 * lat)/V,
			(gps_g.speed/100.0 * lon)/V,
			((gps_g.altitude - gps_g_old.altitude)/100.0)/((last_gps_g-last_gps_g_old)/1000000.0));

	len = mavlink_msg_to_send_buffer(buf, &msg);

	return(send_udpserver(buf, len));
}

/* Send Status */
/*
MAVLINK_MSG_ID_SYS_STATUS   34
 */
int send_mav_status(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	groundcontrol_t mav_g;
	batteria_t bat_g;

	read_batteria_sensori(&bat_g);
	read_groundcontrol(&mav_g);

	//	mavlink_msg_sys_status_pack(airplane.system_id, airplane.component_id, &msg, MAV_MODE_GUIDED, MAV_NAV_HOLD, MAV_STATE_ACTIVE, 500, 7500, 0, 0);
	mavlink_msg_sys_status_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			mav_g.state.mav_mode,
			mav_g.state.nav_mode,
			mav_g.state.status,
			get_avg_cpu_load(), //CPU load 100% = 1000
			bat_g.volt,
			bat_g.stato/10, //battery_remaining Remaining battery energy: (0%: 0, 100%: 1000)
			get_drop_rate());

	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Status RC Channels RAW */
/*
MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
 */
int send_mav_rc_channels_raw(void) {
	int len, i;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	uint16_t servi_out[8];
	attuatori_t attuatori_g;

	get_attuatori(&attuatori_g);

	for (i=0;i<min(8,NUMS_ATTUATORI);i++) {
		servi_out[attuatori_g.id[i]]=to_rc(attuatori_g.value[i]);
	}

	mavlink_msg_rc_channels_raw_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			servi_out[0],
			servi_out[1],
			servi_out[2],
			servi_out[3],
			servi_out[4],
			servi_out[5],
			servi_out[6],
			servi_out[7],
			get_rssi());

	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Status RC Channels Scaled */
/*
MAVLINK_MSG_ID_RC_CHANNELS_SCALED 36
 */
//attenzione sembra non piacere a qgroundstation su windows
int send_mav_rc_channels_scaled(void) {
	int len, i;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	int16_t servi_out[8];
	attuatori_t attuatori_g;

	get_attuatori(&attuatori_g);

	for (i=0;i<min(8,NUMS_ATTUATORI);i++) {
		servi_out[attuatori_g.id[i]]=attuatori_g.value[i];
	}

	mavlink_msg_rc_channels_scaled_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			servi_out[0],
			servi_out[1],
			servi_out[2],
			servi_out[3],
			servi_out[4],
			servi_out[5],
			servi_out[6],
			servi_out[7],
			get_rssi());

	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Status SERVO RAW */
/*
MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 37
 */
int send_mav_servo_output_raw(void) {
	int len, i;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	uint16_t servi_out[8];
	attuatori_t attuatori_g;

	get_attuatori(&attuatori_g);

	for (i=0;i<min(8,NUMS_ATTUATORI);i++) {
		servi_out[attuatori_g.id[i]]=to_rc(attuatori_g.value[i]);
	}

	mavlink_msg_servo_output_raw_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			servi_out[0],
			servi_out[1],
			servi_out[2],
			servi_out[3],
			servi_out[4],
			servi_out[5],
			servi_out[6],
			servi_out[7]);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Control Status */
/*
MAVLINK_MSG_ID_CONTROL_STATUS 52
 */
int send_mav_control_status(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH], gps_fix=0;
	gps_t gps_g;

	read_gps_sensori(&gps_g);

	if (gps_g.GPS_fix) {
		if ((gps_g.gps_sat >= 1)) {
			gps_fix = 1;
		}

		if ((gps_g.gps_sat >= 3)) {
			gps_fix = 2;
		}

		if ((gps_g.gps_sat >= 5)) {
			gps_fix = 3;
		}
	}

	mavlink_msg_control_status_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			get_sys_state_position_fix(),
			0,
			gps_fix,
			255,
			1,
			1,
			1,
			1);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Action Ack */
/*
MAVLINK_MSG_ID_ACTION_ACK   62
 */
int send_mav_action_ack(uint8_t action, uint8_t result) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_action_ack_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			action,
			result);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Global Position (int) */
/*
MAVLINK_MSG_ID_GLOBAL_POSITION_INT 73
 */
int send_mav_global_position_int(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	gps_t gps_g;
	uint64_t last_gps_g;
	float lat,lon;
	double V;

	read_gps_sensori(&gps_g);
	last_gps_g = microsSinceEpoch();

	lat = ((gps_g.latitude - gps_g_old.latitude)*1.0)/(last_gps_g - last_gps_g_old);
	lon = ((gps_g.longitude - gps_g_old.longitude)*1.0)/(last_gps_g - last_gps_g_old);
	if ((V = sqrt(pow(lat,2.0)+pow(lon,2.0)))==0) V=0.0000001;

	mavlink_msg_global_position_int_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			gps_g.latitude,
			gps_g.longitude,
			gps_g.altitude * 10,
			(int16_t)(gps_g.speed * lat)/V,
			(int16_t)(gps_g.speed * lon)/V,
			(int16_t)(gps_g.altitude - gps_g_old.altitude)/((last_gps_g-last_gps_g_old)/1000000.0));

	len = mavlink_msg_to_send_buffer(buf, &msg);

	return(send_udpserver(buf, len));
}

/* Send vfr_hud */
/*
MAVLINK_MSG_ID_VFR_HUD 74
 */
int send_mav_vfr_hud(void) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	uint64_t last_gps_g;
	gps_t gps_g;
	imu_t imu_g;

	read_gps_sensori(&gps_g);
	read_imu_sensori(&imu_g);
	last_gps_g = microsSinceEpoch();

	mavlink_msg_vfr_hud_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			gps_g.speed/100.0,
			gps_g.speed/100.0,
			imu_g.yaw/100,
			get_pilota_throttle_0100()/100,
			gps_g.altitude/100.0,
			((gps_g.altitude - gps_g_old.altitude)/100.0)/((last_gps_g - last_gps_g_old)/1000000.0));

	len = mavlink_msg_to_send_buffer(buf, &msg);

	return(send_udpserver(buf, len));
}

/* Send Command Ack */
/*
MAVLINK_MSG_ID_COMMAND_ACK 76
 */
int send_mav_command_ack(float command, float result) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_command_ack_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			command,
			result);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}

/* Send Status Text */
/*
MAVLINK_MSG_ID_STATUSTEXT   254
 */
int send_mav_statustext(uint8_t severity, uint8_t *text, int dim) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];
	uint8_t texttosend[50];

	if (dim > 50) {
		dim = 50;
	}

	memset(texttosend, 0, 50);
	memcpy(texttosend, text, dim);

	mavlink_msg_statustext_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			severity,
			texttosend);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return(send_udpserver(buf, len));
}


//UTILITY

uint64_t microsSinceEpoch() {
	struct timeval tv;

	uint64_t micros = 0;

	gettimeofday(&tv, NULL);
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

	return (micros);
}

/////////////////////////////////////////////////////////////////////////////////////
//PARAMETER
void param_defaults(void) {
	pthread_mutex_lock(&groundcontrol_mutex);
	//	memset(&groundcontrol_data, 0, sizeof(groundcontrol_t));

	groundcontrol_data.state.mav_mode = MAV_MODE_UNINIT;
	groundcontrol_data.state.nav_mode = MAV_NAV_GROUNDED;
	groundcontrol_data.state.type = MAV_QUADROTOR;
	groundcontrol_data.state.autopilot = MAV_AUTOPILOT_GENERIC;
	groundcontrol_data.state.status = MAV_STATE_UNINIT;

	groundcontrol_data.param[PARAM_SYSTEM_ID] = 1;
	strcpy(groundcontrol_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");

	groundcontrol_data.param[PARAM_COMPONENT_ID] = MAV_COMP_ID_IMU;
	strcpy(groundcontrol_data.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");

	groundcontrol_data.param[PARAM_SYSTEM_TYPE] = MAV_QUADROTOR;
	strcpy(groundcontrol_data.param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");

	groundcontrol_data.param[PARAM_AUTOPILOT_TYPE] = MAV_AUTOPILOT_GENERIC;	//Generic
	strcpy(groundcontrol_data.param_name[PARAM_AUTOPILOT_TYPE], "SYS_AP_TYPE");

	groundcontrol_data.param[PARAM_SW_VERSION] = 2000;
	strcpy(groundcontrol_data.param_name[PARAM_SW_VERSION], "SYS_SW_VER");

	groundcontrol_data.param[PARAM_IMU_RESET] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_IMU_RESET], "SYS_IMU_RESET");

	groundcontrol_data.param[PARAM_ESC_CALIBRATION] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_ESC_CALIBRATION], "BOOT_ESC_CALIBR");

	groundcontrol_data.param[PARAM_PID_ALTI_KP] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ALTI_KP], "ALTI_KP");
	groundcontrol_data.param[PARAM_PID_ALTI_KI] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ALTI_KI], "ALTI_KI");
	groundcontrol_data.param[PARAM_PID_ALTI_KD] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ALTI_KD], "ALTI_KD");
	groundcontrol_data.param[PARAM_PID_ALTI_INTMAX] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ALTI_INTMAX], "ALTI_IMAX");

	groundcontrol_data.param[PARAM_PID_ROLL_KP] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ROLL_KP], "ROLL_KP");
	groundcontrol_data.param[PARAM_PID_ROLL_KI] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ROLL_KI], "ROLL_KI");
	groundcontrol_data.param[PARAM_PID_ROLL_KD] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ROLL_KD], "ROLL_KD");
	groundcontrol_data.param[PARAM_PID_ROLL_INTMAX] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_ROLL_INTMAX], "ROLL_IMAX");

	groundcontrol_data.param[PARAM_PID_PITCH_KP] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_PITCH_KP], "PITCH_KP");
	groundcontrol_data.param[PARAM_PID_PITCH_KI] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_PITCH_KI], "PITCH_KI");
	groundcontrol_data.param[PARAM_PID_PITCH_KD] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_PITCH_KD], "PITCH_KD");
	groundcontrol_data.param[PARAM_PID_PITCH_INTMAX] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_PITCH_INTMAX], "PITCH_IMAX");

	groundcontrol_data.param[PARAM_PID_YAW_KP] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_YAW_KP], "YAW_KP");
	groundcontrol_data.param[PARAM_PID_YAW_KI] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_YAW_KI], "YAW_KI");
	groundcontrol_data.param[PARAM_PID_YAW_KD] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_YAW_KD], "YAW_KD");
	groundcontrol_data.param[PARAM_PID_YAW_INTMAX] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_PID_YAW_INTMAX], "YAW_IMAX");

	groundcontrol_data.param[PARAM_SERVO_0_ID] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_0_ID], "SERVO_0_ID");
	groundcontrol_data.param[PARAM_SERVO_0_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_0_MIN], "SERVO_0_MIN");
	groundcontrol_data.param[PARAM_SERVO_0_ZERO] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_0_ZERO], "SERVO_0_ZERO");
	groundcontrol_data.param[PARAM_SERVO_0_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_0_MAX], "SERVO_0_MAX");

	groundcontrol_data.param[PARAM_SERVO_1_ID] = 1.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_1_ID], "SERVO_1_ID");
	groundcontrol_data.param[PARAM_SERVO_1_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_1_MIN], "SERVO_1_MIN");
	groundcontrol_data.param[PARAM_SERVO_1_ZERO] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_1_ZERO], "SERVO_1_ZERO");
	groundcontrol_data.param[PARAM_SERVO_1_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_1_MAX], "SERVO_1_MAX");

	groundcontrol_data.param[PARAM_SERVO_2_ID] = 2.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_2_ID], "SERVO_2_ID");
	groundcontrol_data.param[PARAM_SERVO_2_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_2_MIN], "SERVO_2_MIN");
	groundcontrol_data.param[PARAM_SERVO_2_ZERO] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_2_ZERO], "SERVO_2_ZERO");
	groundcontrol_data.param[PARAM_SERVO_2_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_2_MAX], "SERVO_2_MAX");

	groundcontrol_data.param[PARAM_SERVO_3_ID] = 3.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_3_ID], "SERVO_3_ID");
	groundcontrol_data.param[PARAM_SERVO_3_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_3_MIN], "SERVO_3_MIN");
	groundcontrol_data.param[PARAM_SERVO_3_ZERO] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_3_ZERO], "SERVO_3_ZERO");
	groundcontrol_data.param[PARAM_SERVO_3_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_3_MAX], "SERVO_3_MAX");

	groundcontrol_data.param[PARAM_SERVO_4_ID] = 4.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_4_ID], "SERVO_4_ID");
	groundcontrol_data.param[PARAM_SERVO_4_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_4_MIN], "SERVO_4_MIN");
	groundcontrol_data.param[PARAM_SERVO_4_ZERO] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_4_ZERO], "SERVO_4_ZERO");
	groundcontrol_data.param[PARAM_SERVO_4_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_4_MAX], "SERVO_4_MAX");

	groundcontrol_data.param[PARAM_SERVO_5_ID] = 5.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_5_ID], "SERVO_5_ID");
	groundcontrol_data.param[PARAM_SERVO_5_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_5_MIN], "SERVO_5_MIN");
	groundcontrol_data.param[PARAM_SERVO_5_ZERO] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_5_ZERO], "SERVO_5_ZERO");
	groundcontrol_data.param[PARAM_SERVO_5_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_5_MAX], "SERVO_5_MAX");

	groundcontrol_data.param[PARAM_SERVO_6_ID] = 6.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_6_ID], "SERVO_6_ID");
	groundcontrol_data.param[PARAM_SERVO_6_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_6_MIN], "SERVO_6_MIN");
	groundcontrol_data.param[PARAM_SERVO_6_ZERO] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_6_ZERO], "SERVO_6_ZERO");
	groundcontrol_data.param[PARAM_SERVO_6_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_6_MAX], "SERVO_6_MAX");

	groundcontrol_data.param[PARAM_SERVO_7_ID] = 7.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_7_ID], "SERVO_7_ID");
	groundcontrol_data.param[PARAM_SERVO_7_MIN] = -10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_7_MIN], "SERVO_7_MIN");
	groundcontrol_data.param[PARAM_SERVO_7_ZERO] = 0.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_7_ZERO], "SERVO_7_ZERO");
	groundcontrol_data.param[PARAM_SERVO_7_MAX] = 10000.0;
	strcpy(groundcontrol_data.param_name[PARAM_SERVO_7_MAX], "SERVO_7_MAX");

	pthread_mutex_unlock(&groundcontrol_mutex);
}

void param_read_all(void) {
	int res;
	FILE *fp = NULL;
	groundcontrol_t parametri;

	read_groundcontrol(&parametri);

	if ((fp = fopen("params.txt", "r")) != NULL) {
		res = fread(&(parametri.param), sizeof(float) * ONBOARD_PARAM_COUNT, 1, fp);
		res = fread(&(parametri.param_name), sizeof(char) * ONBOARD_PARAM_COUNT * ONBOARD_PARAM_NAME_LENGTH, 1, fp);


		write_groundcontrol(&parametri);

		fclose(fp);
	}
}

void param_write_all(void) {
	int res;
	FILE *fp = NULL;
	groundcontrol_t parametri;

	read_groundcontrol(&parametri);

	if ((fp = fopen("params.txt", "w")) != NULL) {
		res = fwrite(&(parametri.param), sizeof(float) * ONBOARD_PARAM_COUNT, 1, fp);
		res = fwrite(&(parametri.param_name), sizeof(char) * ONBOARD_PARAM_COUNT * ONBOARD_PARAM_NAME_LENGTH, 1, fp);


		fclose(fp);
	}

}

void handle_param(int i){
	switch(i){
	case PARAM_SYSTEM_ID:
		//niente
		break;
	case PARAM_COMPONENT_ID:
		//niente
		break;
	case PARAM_SYSTEM_TYPE:
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.type = groundcontrol_data.param[PARAM_SYSTEM_TYPE];
		pthread_mutex_unlock(&groundcontrol_mutex);
		break;
	case PARAM_AUTOPILOT_TYPE:
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.autopilot = groundcontrol_data.param[PARAM_AUTOPILOT_TYPE];
		pthread_mutex_unlock(&groundcontrol_mutex);
		break;
	case PARAM_SW_VERSION:
		//niente
		break;
	case PARAM_IMU_RESET:
		//handler
		break;
	case PARAM_ESC_CALIBRATION:
		break;

	case PARAM_PID_ALTI_KP:
	case PARAM_PID_ALTI_KI:
	case PARAM_PID_ALTI_KD:
	case PARAM_PID_ALTI_INTMAX:
		pid_set_parameters(&altitudePID,
				groundcontrol_data.param[PARAM_PID_ALTI_KP],
				groundcontrol_data.param[PARAM_PID_ALTI_KI],
				groundcontrol_data.param[PARAM_PID_ALTI_KD],
				groundcontrol_data.param[PARAM_PID_ALTI_INTMAX]);
		break;

	case PARAM_PID_ROLL_KP:
	case PARAM_PID_ROLL_KI:
	case PARAM_PID_ROLL_KD:
	case PARAM_PID_ROLL_INTMAX:
		pid_set_parameters(&rollPID,
				groundcontrol_data.param[PARAM_PID_ROLL_KP],
				groundcontrol_data.param[PARAM_PID_ROLL_KI],
				groundcontrol_data.param[PARAM_PID_ROLL_KD],
				groundcontrol_data.param[PARAM_PID_ROLL_INTMAX]);
		break;

	case PARAM_PID_PITCH_KP:
	case PARAM_PID_PITCH_KI:
	case PARAM_PID_PITCH_KD:
	case PARAM_PID_PITCH_INTMAX:
		pid_set_parameters(&pitchPID,
				groundcontrol_data.param[PARAM_PID_PITCH_KP],
				groundcontrol_data.param[PARAM_PID_PITCH_KI],
				groundcontrol_data.param[PARAM_PID_PITCH_KD],
				groundcontrol_data.param[PARAM_PID_PITCH_INTMAX]);
		break;

	case PARAM_PID_YAW_KP:
	case PARAM_PID_YAW_KI:
	case PARAM_PID_YAW_KD:
	case PARAM_PID_YAW_INTMAX:
		pid_set_parameters(&yawPID,
				groundcontrol_data.param[PARAM_PID_YAW_KP],
				groundcontrol_data.param[PARAM_PID_YAW_KI],
				groundcontrol_data.param[PARAM_PID_YAW_KD],
				groundcontrol_data.param[PARAM_PID_YAW_INTMAX]);
		break;

	case PARAM_SERVO_0_ID:
	case PARAM_SERVO_0_MIN:
	case PARAM_SERVO_0_ZERO:
	case PARAM_SERVO_0_MAX: {
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_0_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_0_ZERO], groundcontrol_data.param[PARAM_SERVO_0_MIN], groundcontrol_data.param[PARAM_SERVO_0_MAX]);
		attuatori_g.id[0] = groundcontrol_data.param[PARAM_SERVO_0_ID];
		attuatori_g.min[0] = groundcontrol_data.param[PARAM_SERVO_0_MIN];
		attuatori_g.zero[0] = groundcontrol_data.param[PARAM_SERVO_0_ZERO];
		attuatori_g.max[0] = groundcontrol_data.param[PARAM_SERVO_0_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;

	case PARAM_SERVO_1_ID:
	case PARAM_SERVO_1_MIN:
	case PARAM_SERVO_1_ZERO:
	case PARAM_SERVO_1_MAX: {
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_1_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_1_ZERO], groundcontrol_data.param[PARAM_SERVO_1_MIN], groundcontrol_data.param[PARAM_SERVO_1_MAX]);
		attuatori_g.id[1] = groundcontrol_data.param[PARAM_SERVO_1_ID];
		attuatori_g.min[1] = groundcontrol_data.param[PARAM_SERVO_1_MIN];
		attuatori_g.zero[1] = groundcontrol_data.param[PARAM_SERVO_1_ZERO];
		attuatori_g.max[1] = groundcontrol_data.param[PARAM_SERVO_1_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;

	case PARAM_SERVO_2_ID:
	case PARAM_SERVO_2_MIN:
	case PARAM_SERVO_2_ZERO:
	case PARAM_SERVO_2_MAX: {
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_2_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_2_ZERO], groundcontrol_data.param[PARAM_SERVO_2_MIN], groundcontrol_data.param[PARAM_SERVO_2_MAX]);
		attuatori_g.id[2] = groundcontrol_data.param[PARAM_SERVO_2_ID];
		attuatori_g.min[2] = groundcontrol_data.param[PARAM_SERVO_2_MIN];
		attuatori_g.zero[2] = groundcontrol_data.param[PARAM_SERVO_2_ZERO];
		attuatori_g.max[2] = groundcontrol_data.param[PARAM_SERVO_2_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;

	case PARAM_SERVO_3_ID:
	case PARAM_SERVO_3_MIN:
	case PARAM_SERVO_3_ZERO:
	case PARAM_SERVO_3_MAX: {
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_3_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_3_ZERO], groundcontrol_data.param[PARAM_SERVO_3_MIN], groundcontrol_data.param[PARAM_SERVO_3_MAX]);
		attuatori_g.id[3] = groundcontrol_data.param[PARAM_SERVO_3_ID];
		attuatori_g.min[3] = groundcontrol_data.param[PARAM_SERVO_3_MIN];
		attuatori_g.zero[3] = groundcontrol_data.param[PARAM_SERVO_3_ZERO];
		attuatori_g.max[3] = groundcontrol_data.param[PARAM_SERVO_3_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;

	case PARAM_SERVO_4_ID:
	case PARAM_SERVO_4_MIN:
	case PARAM_SERVO_4_ZERO:
	case PARAM_SERVO_4_MAX:{
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_4_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_4_ZERO], groundcontrol_data.param[PARAM_SERVO_4_MIN], groundcontrol_data.param[PARAM_SERVO_4_MAX]);
		attuatori_g.id[4] = groundcontrol_data.param[PARAM_SERVO_4_ID];
		attuatori_g.min[4] = groundcontrol_data.param[PARAM_SERVO_4_MIN];
		attuatori_g.zero[4] = groundcontrol_data.param[PARAM_SERVO_4_ZERO];
		attuatori_g.max[4] = groundcontrol_data.param[PARAM_SERVO_4_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;

	case PARAM_SERVO_5_ID:
	case PARAM_SERVO_5_MIN:
	case PARAM_SERVO_5_ZERO:
	case PARAM_SERVO_5_MAX:{
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_5_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_5_ZERO], groundcontrol_data.param[PARAM_SERVO_5_MIN], groundcontrol_data.param[PARAM_SERVO_5_MAX]);
		attuatori_g.id[5] = groundcontrol_data.param[PARAM_SERVO_5_ID];
		attuatori_g.min[5] = groundcontrol_data.param[PARAM_SERVO_5_MIN];
		attuatori_g.zero[5] = groundcontrol_data.param[PARAM_SERVO_5_ZERO];
		attuatori_g.max[5] = groundcontrol_data.param[PARAM_SERVO_5_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;

	case PARAM_SERVO_6_ID:
	case PARAM_SERVO_6_MIN:
	case PARAM_SERVO_6_ZERO:
	case PARAM_SERVO_6_MAX: {
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_6_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_6_ZERO], groundcontrol_data.param[PARAM_SERVO_6_MIN], groundcontrol_data.param[PARAM_SERVO_6_MAX]);
		attuatori_g.id[6] = groundcontrol_data.param[PARAM_SERVO_6_ID];
		attuatori_g.min[6] = groundcontrol_data.param[PARAM_SERVO_6_MIN];
		attuatori_g.zero[6] = groundcontrol_data.param[PARAM_SERVO_6_ZERO];
		attuatori_g.max[6] = groundcontrol_data.param[PARAM_SERVO_6_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;

	case PARAM_SERVO_7_ID:
	case PARAM_SERVO_7_MIN:
	case PARAM_SERVO_7_ZERO:
	case PARAM_SERVO_7_MAX: {
		attuatori_t attuatori_g;

		get_attuatori(&attuatori_g);
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.param[PARAM_SERVO_7_ZERO] = clamp(groundcontrol_data.param[PARAM_SERVO_7_ZERO], groundcontrol_data.param[PARAM_SERVO_7_MIN], groundcontrol_data.param[PARAM_SERVO_7_MAX]);
		attuatori_g.id[7] = groundcontrol_data.param[PARAM_SERVO_7_ID];
		attuatori_g.min[7] = groundcontrol_data.param[PARAM_SERVO_7_MIN];
		attuatori_g.zero[7] = groundcontrol_data.param[PARAM_SERVO_7_ZERO];
		attuatori_g.max[7] = groundcontrol_data.param[PARAM_SERVO_7_MAX];
		pthread_mutex_unlock(&groundcontrol_mutex);
		set_attuatori(&attuatori_g);
	}
	break;
	}
}

float get_param_value(uint16_t param_id) {
	float res;

	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.param[param_id];
	pthread_mutex_unlock(&groundcontrol_mutex);
	return(res);
}
int set_param_value(uint16_t param_id, float value) {
	int res = -1;
	pthread_mutex_lock(&groundcontrol_mutex);
	if ((groundcontrol_data.param[param_id] != value) && (!isnan(value)) && (!isinf(value))) {
		groundcontrol_data.param[param_id] = value;
		res = 0;
	}
	pthread_mutex_unlock(&groundcontrol_mutex);
	return(res);
}
int set_param_value_with_name(uint8_t *param_id, float value) {
	int i, j;
	char* key = (char*) param_id;
	char  test[15];

	for (i = 0; i < ONBOARD_PARAM_COUNT; i++) {
		bool match = true;
		memset(test, 0, ONBOARD_PARAM_NAME_LENGTH);
		get_param_name(test, i);
		for (j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++) {
			// Compare
			if (((char) (test[j])) != (char) (key[j])) {
				match = false;
			}

			// End matching if null termination is reached
			if ((char) (test[j])== '\0') {
				break;
			}
		}

		// Check if matched
		if (match) {
			set_param_value(i, value);
			return(i);
		}
	}

	return(-1);
}

int get_param_id_with_name(uint8_t *param_id) {
	int i, j;
	char* key = (char*) param_id;
	char  test[15];

	for (i = 0; i < ONBOARD_PARAM_COUNT; i++) {
		bool match = true;
		memset(test, 0, ONBOARD_PARAM_NAME_LENGTH);
		get_param_name(test, i);
		for (j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++) {
			// Compare
			if (((char) (test[j])) != (char) (key[j])) {
				match = false;
			}

			// End matching if null termination is reached
			if ((char) (test[j])== '\0') {
				break;
			}
		}

		// Check if matched
		if (match) {
			return(i);
		}
	}

	return(-1);
}

char *get_param_name(char *name, uint16_t param_id) {
	pthread_mutex_lock(&groundcontrol_mutex);
	memcpy(name, groundcontrol_data.param_name[param_id], ONBOARD_PARAM_NAME_LENGTH);
	pthread_mutex_unlock(&groundcontrol_mutex);

	return(name);
}

/////////////////////////////////////////////////////////////////////////////////////
//SYS_STATE
void set_datastream(enum MAV_DATA_STREAM dstream, uint16_t rate, bool enable) {
	pthread_mutex_lock(&groundcontrol_mutex);
	groundcontrol_data.datastream[dstream].enable = enable;
	groundcontrol_data.datastream[dstream].rate = min(rate, 50);
	pthread_mutex_unlock(&groundcontrol_mutex);
}

/** @return Get the current mode of operation */
enum MAV_MODE get_sys_state_mode(void) {
	enum MAV_MODE res;

	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.state.mav_mode;
	pthread_mutex_unlock(&groundcontrol_mutex);

	return (res);
}
/** @brief Set the current mode of operation */
bool set_sys_state_mode(enum MAV_MODE mode) {
	if (mode == MAV_MODE_AUTO) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_AUTO;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_GUIDED) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_GUIDED;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_LOCKED) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_LOCKED;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_MANUAL) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_MANUAL;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_READY) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_READY;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_TEST1) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_TEST1;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_TEST2) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_TEST2;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_TEST3) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.mav_mode = MAV_MODE_TEST3;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} else if (mode == MAV_MODE_RC_TRAINING) {
		// Only go into RC training if not flying
		if (!sys_state_is_flying()) {
			pthread_mutex_lock(&groundcontrol_mutex);
			groundcontrol_data.state.mav_mode = MAV_MODE_RC_TRAINING;
			pthread_mutex_unlock(&groundcontrol_mutex);
			return (true);
		} else {
			perror("WARNING: SYSTEM IS IN FLIGHT! Denied to switch to RC mode");
			return (false);
		}
	} else {
		// UNINIT is not a mode that should be actively set
		// it will thus be rejected like any other invalid mode

		// No valid mode
		perror("WARNING: Attempted to set invalid mode");
		return (false);
	}
}

/** @brief Get the current navigation mode */
enum MAV_NAV get_sys_state_nav_mode(void) {
	enum MAV_NAV res;
	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.state.nav_mode;
	pthread_mutex_unlock(&groundcontrol_mutex);
	return(res);
}
/** @brief Set the current navigation mode */
void set_sys_state_nav_mode(enum MAV_NAV nav_mode){
	pthread_mutex_lock(&groundcontrol_mutex);
	if (groundcontrol_data.state.nav_mode != nav_mode) {
		groundcontrol_data.state.prevnav_mode = groundcontrol_data.state.nav_mode;
	}
	groundcontrol_data.state.nav_mode = nav_mode;
	pthread_mutex_unlock(&groundcontrol_mutex);
}
void set_sys_state_nav_mode_prev(){
	pthread_mutex_lock(&groundcontrol_mutex);
	groundcontrol_data.state.nav_mode = groundcontrol_data.state.prevnav_mode;
	pthread_mutex_unlock(&groundcontrol_mutex);
}

/** @brief Get the current system type */
enum MAV_TYPE get_sys_state_type(void);
/** @brief Set the current system type */
void set_sys_state_type(enum MAV_TYPE type){
	pthread_mutex_lock(&groundcontrol_mutex);
	groundcontrol_data.state.type = type;
	pthread_mutex_unlock(&groundcontrol_mutex);
}

/** @brief Get the current autopilot type */
enum MAV_AUTOPILOT_TYPE get_sys_state_autopilot(void);
/** @brief Set the current autopilot type */
void set_sys_state_autopilot(enum MAV_AUTOPILOT_TYPE type);

/** @brief Get the current system state */
enum MAV_STATE get_sys_state_status(void) {
	enum MAV_STATE res;

	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.state.status;
	pthread_mutex_unlock(&groundcontrol_mutex);

	return (res);
}
/** @brief Set the current system state */
bool set_sys_state_status(enum MAV_STATE state) {
	bool res = false;
	if (state == MAV_STATE_ACTIVE) {
		pthread_mutex_lock(&groundcontrol_mutex);
		if ((groundcontrol_data.state.status == MAV_STATE_ACTIVE) ||
				(groundcontrol_data.state.status == MAV_STATE_STANDBY) ||
				(groundcontrol_data.state.status == MAV_STATE_CRITICAL) ||
				(groundcontrol_data.state.status == MAV_STATE_EMERGENCY)) {
			groundcontrol_data.state.prevstatus = groundcontrol_data.state.status;
			groundcontrol_data.state.status = MAV_STATE_ACTIVE;
			res = true;
		}
		pthread_mutex_unlock(&groundcontrol_mutex);
	} else if (state == MAV_STATE_BOOT) {
		pthread_mutex_lock(&groundcontrol_mutex);
		if (groundcontrol_data.state.status == MAV_STATE_UNINIT) {
			groundcontrol_data.state.prevstatus = groundcontrol_data.state.status;
			groundcontrol_data.state.status = MAV_STATE_BOOT;
			res = true;
		}
		pthread_mutex_unlock(&groundcontrol_mutex);
	} else if (state == MAV_STATE_CALIBRATING) {
		pthread_mutex_lock(&groundcontrol_mutex);
		if (groundcontrol_data.state.status == MAV_STATE_STANDBY) {
			groundcontrol_data.state.prevstatus = groundcontrol_data.state.status;
			groundcontrol_data.state.status = MAV_STATE_CALIBRATING;
			res = true;
		}
		pthread_mutex_unlock(&groundcontrol_mutex);
	} else if (state == MAV_STATE_CRITICAL) {
		pthread_mutex_lock(&groundcontrol_mutex);
		if (groundcontrol_data.state.status == MAV_STATE_ACTIVE) {
			groundcontrol_data.state.prevstatus = groundcontrol_data.state.status;
			groundcontrol_data.state.status = MAV_STATE_CRITICAL;
			res = true;
		}
		pthread_mutex_unlock(&groundcontrol_mutex);
	} else if (state == MAV_STATE_EMERGENCY) {
		pthread_mutex_lock(&groundcontrol_mutex);
		if ((groundcontrol_data.state.status == MAV_STATE_ACTIVE) ||
				(groundcontrol_data.state.status == MAV_STATE_CRITICAL)) {
			groundcontrol_data.state.prevstatus = groundcontrol_data.state.status;
			groundcontrol_data.state.status = MAV_STATE_EMERGENCY;
			res = true;
		}
		pthread_mutex_unlock(&groundcontrol_mutex);
	} else if (state == MAV_STATE_POWEROFF) {
		pthread_mutex_lock(&groundcontrol_mutex);
		if (groundcontrol_data.state.status == MAV_STATE_STANDBY) {
			groundcontrol_data.state.prevstatus = groundcontrol_data.state.status;
			groundcontrol_data.state.status = MAV_STATE_POWEROFF;
			res = true;
		}
		pthread_mutex_unlock(&groundcontrol_mutex);
	} else if (state == MAV_STATE_STANDBY) {
		pthread_mutex_lock(&groundcontrol_mutex);
		if ((groundcontrol_data.state.status == MAV_STATE_BOOT) ||
				(groundcontrol_data.state.status == MAV_STATE_CALIBRATING) ||
				(groundcontrol_data.state.status == MAV_STATE_ACTIVE)) {
			groundcontrol_data.state.prevstatus = groundcontrol_data.state.status;
			groundcontrol_data.state.status = MAV_STATE_STANDBY;
			res = true;
		}
		pthread_mutex_unlock(&groundcontrol_mutex);
	} /*else if (state == MAV_STATE_UNINIT) {
		pthread_mutex_lock(&groundcontrol_mutex);
		groundcontrol_data.state.status = MAV_STATE_BOOT;
		pthread_mutex_unlock(&groundcontrol_mutex);
		return (true);
	} */else {
		// UNINIT or invalid state, ignore value and return false
		send_mav_statustext(50, "WARNING: Attempted to set invalid state", 39);
	}
	return (res);
}

uint8_t get_sys_state_position_fix(void){
	uint8_t res;

	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.state.position_fix;
	pthread_mutex_unlock(&groundcontrol_mutex);

	return(res);
}
void set_sys_state_position_fix(uint8_t position_fix) {
	pthread_mutex_lock(&groundcontrol_mutex);
	groundcontrol_data.state.position_fix = position_fix;
	pthread_mutex_unlock(&groundcontrol_mutex);
}

/**
 * @brief Check if the system is currently in flight mode
 *
 * This function will return false for all states that allow
 * changes to critical system variables, such as calibrating sensors
 * or changing the system type.
 *
 * @return 0 if the system is not flying, 1 if it is flying or flight-ready
 */
bool sys_state_is_flying(void) {
	bool res;

	pthread_mutex_lock(&groundcontrol_mutex);
	if (groundcontrol_data.state.status == MAV_STATE_STANDBY || groundcontrol_data.state.status == MAV_STATE_UNINIT) {
		res = false;
	} else {
		res = true;
	}
	pthread_mutex_unlock(&groundcontrol_mutex);

	return(res);
}
enum FLY_STATE get_sys_state_fly(void){
	//uint8_t get_sys_state_fly(void){
	uint8_t res;

	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.state.fly;
	pthread_mutex_unlock(&groundcontrol_mutex);

	return(res);
}
void set_sys_state_fly(enum FLY_STATE fly) {
	//void set_sys_state_fly(uint8_t fly) {
	pthread_mutex_lock(&groundcontrol_mutex);
	if (groundcontrol_data.state.fly != fly) {
		groundcontrol_data.state.prevfly  = groundcontrol_data.state.fly ;
	}
	groundcontrol_data.state.fly = fly;
	pthread_mutex_unlock(&groundcontrol_mutex);
}

/** @brief Check if the system is indoor flight mode */
bool get_sys_state_is_indoor(void) {
	bool res;

	pthread_mutex_lock(&groundcontrol_mutex);
	res = groundcontrol_data.state.indoor;
	pthread_mutex_unlock(&groundcontrol_mutex);

	return(res);
}


void update_system_statemachine(void) {
	// Update state machine, enable and disable controllers

	switch (get_sys_state_mode()) {
	case MAV_MODE_UNINIT: {
		set_sys_state_nav_mode(MAV_NAV_GROUNDED);
		set_sys_state_status(MAV_STATE_BOOT);
	}
	break;

	case MAV_MODE_LOCKED:
		set_sys_state_nav_mode(MAV_NAV_GROUNDED);
		set_sys_state_status(MAV_STATE_STANDBY);
		break;

	case MAV_MODE_MANUAL:
		//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
		//global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 0;
		//global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
		//global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
		//global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
		//set_sys_state_nav_mode(MAV_NAV_FREE_DRIFT);
		//set_sys_state_status(MAV_STATE_ACTIVE);
		break;
		/*
	case MAV_MODE_GUIDED: {

		if (get_sys_state_position_fix()) {
			if (get_sys_state_status() == MAV_STATE_CRITICAL) {
				//get active again if we are in critical
				//if we are in Emergency we don't want to start again!!!!
				set_sys_state_status(MAV_STATE_ACTIVE);
			}
		} else {
			if (get_sys_state_status() == MAV_STATE_ACTIVE) {
				set_sys_state_status(MAV_STATE_CRITICAL);
				//				global_data.entry_critical = loop_start_time;
				//				debug_message_buffer("CRITICAL! POSITION LOST");
			} else if ((get_sys_state_status() == MAV_STATE_CRITICAL)) {
				//					&& (loop_start_time - global_data.entry_critical > 3 * (uint32_t) global_data.param[PARAM_POSITION_TIMEOUT])) {
				//wait 3 times as long as waited for critical

				set_sys_state_status(MAV_STATE_EMERGENCY);
				//Initiate Landing even if we didn't reach 0.7m
				set_sys_state_fly(FLY_LANDING);
				//				debug_message_buffer("EMERGENCY! MAYDAY! MAYDAY! LANDING!");
			}
		}
		switch (get_sys_state_status()) {
		case MAV_STATE_ACTIVE:
			//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 1;
			//global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
			//global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 1;
			break;

		case MAV_STATE_CRITICAL:
			////global_data.param[PARAM_MIX_POSITION_WEIGHT] = 1; // estimate path without vision
			//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0; // try to hover on place
			//global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
			//global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 1;
			break;

		case MAV_STATE_EMERGENCY:
			//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
			//global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
			//global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 1;
			break;
		default:
			printf("");
			//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
			//global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 0;
			//global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
		}

		//global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
		//global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;

	}
	break;
	case MAV_MODE_AUTO:
		// Same as guided mode, NO BREAAK
		break;
	case MAV_MODE_TEST1:
		// Same as guided mode, NO BRE
		break;
	case MAV_MODE_TEST2:
		//allow other mix params to be set by hand
		//global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
		//global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
		break;
	case MAV_MODE_TEST3:
		break;
	case MAV_MODE_READY:
		break;
		 */
	default:
		printf("");
		//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
		//global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 0;
		//global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
		//global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 0;
		//global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 0;
	}
	//	if (global_data.state.remote_ok == 0) {
	//global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 0;
	//	}
}

/** @brief Get the current manual crtl */
manual_ctrl_t get_sys_state_manual_ctrl(void) {
	manual_ctrl_t mctrl;
	pthread_mutex_lock(&groundcontrol_mutex);
	memcpy(&mctrl, &groundcontrol_data.state.remote, sizeof(manual_ctrl_t));
	pthread_mutex_unlock(&groundcontrol_mutex);
	return(mctrl);
}
/** @brief Set the current manual crtl */
bool set_sys_state_manual_ctrl(manual_ctrl_t mctrl) {
	bool res = true;
	pthread_mutex_lock(&groundcontrol_mutex);
	memcpy(&groundcontrol_data.state.remote, &mctrl, sizeof(manual_ctrl_t));
	pthread_mutex_unlock(&groundcontrol_mutex);
	return(res);
}

/////////////////////////////////////////////////////////////////////////////////////
//RADIO
void esc_calibration(void) {
	int i;
	attuatori_t attuatori_g;

	get_attuatori(&attuatori_g);

	for(i=0;i<NUMS_ATTUATORI;i++) {
		if((attuatori_g.id[i]==0) || (attuatori_g.id[i]==1) || (attuatori_g.id[i]==2) || (attuatori_g.id[i]==3)) {
			attuatori_g.value[i] = attuatori_g.max[i];
		}
	}

	write_attuatori(&attuatori_g);

	sleep(3);

	for(i=0;i<NUMS_ATTUATORI;i++) {
		if((attuatori_g.id[i]==0) || (attuatori_g.id[i]==1) || (attuatori_g.id[i]==2) || (attuatori_g.id[i]==3)) {
			attuatori_g.value[i] = attuatori_g.min[i];
		}
	}
	write_attuatori(&attuatori_g);

	sleep(10);
}

//re-map rc in 0-10000 value 0->0% 10000->100%
int16_t to_10000(float value) {
	return ((uint16_t)remap(value, 900, 2100, -10000, 10000));
}
uint16_t to_rc(int16_t value) {
	return (remap(value, -10000.0, 10000.0, 900.0, 2100.0));
}

//rssi 0-255
uint8_t get_rssi(void) {
	FILE *fp = NULL;
	uint8_t res = 0, none;

	//	if ((fp=fopen(WLAN_LEVEL, "r")) != NULL) {

	//		none = fscanf(fp, "%c", &res);

	//		fclose(fp);
	//	}

	return(res);
};

uint32_t get_drop_rate(void) {
	uint32_t res;

	pthread_mutex_lock(&groundcontrol_mutex);
	res = ((groundcontrol_data.comm.rx_drop_count*1000+1)/(groundcontrol_data.comm.rx_success_count+1));
	pthread_mutex_unlock(&groundcontrol_mutex);
	return(res);
}
void set_drop_rate(	uint32_t drop_count, uint32_t success_count) {
	pthread_mutex_lock(&groundcontrol_mutex);
	groundcontrol_data.comm.rx_drop_count=drop_count;
	groundcontrol_data.comm.rx_success_count=success_count;
	pthread_mutex_unlock(&groundcontrol_mutex);
}

/////////////////////////////////////////////////////////////////////////////////////
//OTHER
void gps_set_local_origin(void) {
	pilota_location_t pilota_g;
	gps_t gps_g;
	imu_t imu_g;

	read_gps_sensori(&gps_g);
	read_imu_sensori(&imu_g);

	pilota_g.altitude = gps_g.altitude;
	pilota_g.latitude = gps_g.latitude;
	pilota_g.longitude = gps_g.longitude;

	pilota_g.heading = imu_g.yaw;

	storeBase(&pilota_g);
};

int64_t sys_time_clock_get_unix_offset(void) {return (0);};
void sys_time_clock_set_unix_offset(int64_t offset) {}

//CPU usage, 0 = 0%, 1000 = 100%
uint16_t get_avg_cpu_load(void) {
	int processi;
	uint16_t res=0;
	double loadavg[3];

	//loadavg[0]=0;
	processi = getloadavg(loadavg, 3);

	res = clamp(loadavg[0]*1000,0,1000);

	return(res);
}

