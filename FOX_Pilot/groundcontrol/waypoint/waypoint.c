/*
 * waypoint.c
 *
 *  Created on: 01/set/2011
 *      Author: vito
 */

#include "../proto.h"

#include "waypoint.h"

mavlink_waypoint_storage_t waypoint_data;
pthread_mutex_t waypoint_mutex;

void *waypoint_loop (void *ptr) {
	int i;

	init_waypoint();

	while(true){
	    //check for timed-out operations
	    if ((get_waypoint_timestamp_lastaction() > get_waypoint_timeout())
	    		&& (get_waypoint_current_state() != MAVLINK_WPM_STATE_IDLE)) {
	    	set_waypoint_current_state(MAVLINK_WPM_STATE_IDLE);
	    	set_waypoint_current_count(0);
	    	set_waypoint_current_active_wp_id(-1);

	        if(get_waypoint_size() == 0) {
	        	set_waypoint_current_active_wp_id(-1);
	        }
	    }

//	    if(((microsSinceEpoch() - get_waypoint_timestamp_last_send_setpoint()) > get_waypoint_delay_setpoint())
//	    		&& (wpm.current_active_wp_id < wpm.size)) {
//	        send_mav_waypoint_setpoint(sysid, compid, get_waypoint_current_active_wp_id());
//	    }




	    //check if the current waypoint was reached
	    if (get_waypoint_pos_reached() && !get_waypoint_idle()) {
	        if (get_waypoint_current_active_wp_id() < get_waypoint_size()) {
	            mavlink_waypoint_t cur_wp = get_waypoint(get_waypoint_current_active_wp_id());

	            if (get_waypoint_timestamp_firstinside_orbit() == 0) {
	                // Announce that last waypoint was reached

	            	send_mav_waypoint_reached(cur_wp.seq);
	                set_waypoint_timestamp_firstinside_orbit();
	            }

	            // check if the MAV was long enough inside the waypoint orbit
	            //if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))
	            if((microsSinceEpoch() - get_waypoint_timestamp_firstinside_orbit()) >= (cur_wp.param2 * 1000)) {
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
	                    set_waypoint_timestamp_firstinside_orbit();
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




		printf("waypoint_data.size             %d\n",waypoint_data.size);
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
	if (wp.seq<waypoint_data.size) {
		waypoint_data.waypoints[wp.seq] = wp;
	} else {
		res = 1;
	}
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
int set_waypoint_current_active_wp_id(uint16_t wp_id) {
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
uint16_t get_waypoint_current_active_wp_id(void) {
	uint16_t res;
	pthread_mutex_lock(&waypoint_mutex);
	res = waypoint_data.current_wp_id;
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
void set_waypoint_timestamp_firstinside_orbit(void) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.timestamp_firstinside_orbit = microsSinceEpoch();
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

void set_waypoint_waypoints_current(uint16_t wp_id, bool value) {
	pthread_mutex_lock(&waypoint_mutex);
	waypoint_data.waypoints[wp_id].current = (uint8_t)value;
	pthread_mutex_unlock(&waypoint_mutex);
}



///////////////////////////////////////////////////

// Sends an waypoint ack message
void send_mav_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t type) {
	int len;
	mavlink_message_t msg;
	uint8_t buf[MAV_BUFFER_LENGTH];

	mavlink_msg_waypoint_ack_pack(get_param_value(PARAM_SYSTEM_ID),
			get_param_value(PARAM_COMPONENT_ID),
			&msg,
			sysid,
			compid,
			type);

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
				get_param_value(PARAM_COMPONENT_ID),
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
        			get_param_value(PARAM_COMPONENT_ID),
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
			get_param_value(PARAM_COMPONENT_ID),
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

		mavlink_msg_waypoint_pack(sysid,
				compid,
				&msg,
				waypoint.target_system,
				waypoint.target_component,
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
				get_param_value(PARAM_COMPONENT_ID),
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
				get_param_value(PARAM_COMPONENT_ID),
				&msg,
				seq);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		send_udpserver(buf, len);
	}
}

//float mavlink_wpm_distance_to_segment(uint16_t seq, float x, float y, float z) {
//    if (seq < wpm.size) {
//        mavlink_waypoint_t *cur = waypoints->at(seq);
//
//        const PxVector3 A(cur->x, cur->y, cur->z);
//        const PxVector3 C(x, y, z);
//
//        // seq not the second last waypoint
//        if ((uint16_t)(seq+1) < wpm.size) {
//            mavlink_waypoint_t *next = waypoints->at(seq+1);
//            const PxVector3 B(next->x, next->y, next->z);
//            const float r = (B-A).dot(C-A) / (B-A).lengthSquared();
//            if (r >= 0 && r <= 1) {
//                const PxVector3 P(A + r*(B-A));
//                return (P-C).length();
//            } else if (r < 0.f) {
//                return (C-A).length();
//            } else {
//                return (C-B).length();
//            }
//        } else {
//            return (C-A).length();
//        }
//    } else {
//        if (verbose) printf("ERROR: index out of bounds\n");
//    }
//    return -1.f;
//}
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
/*
static void mavlink_wpm_message_handler(const mavlink_message_t* msg) {
    // Handle param messages
    //paramClient->handleMAVLinkPacket(msg);

    //check for timed-out operations
    if (now-wpm.timestamp_lastaction > wpm.timeout && wpm.current_state != MAVLINK_WPM_STATE_IDLE) {
        wpm.current_state = MAVLINK_WPM_STATE_IDLE;
        wpm.current_count = 0;
        wpm.current_partner_sysid = 0;
        wpm.current_partner_compid = 0;
        wpm.current_wp_id = -1;

        if(wpm.size == 0) {
            wpm.current_active_wp_id = -1;
        }
    }

    if(now-wpm.timestamp_last_send_setpoint > wpm.delay_setpoint && wpm.current_active_wp_id < wpm.size) {
        mavlink_wpm_send_setpoint(wpm.current_active_wp_id);
    }

    switch(msg->msgid) {
		case MAVLINK_MSG_ID_ATTITUDE: {
            if(msg->sysid == mavlink_system.sysid && wpm.current_active_wp_id < wpm.size)
            {
                mavlink_waypoint_t *wp = &(wpm.waypoints[wpm.current_active_wp_id]);
                if(wp->frame == MAV_FRAME_LOCAL_ENU || wp->frame == MAV_FRAME_LOCAL_NED)
                {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(msg, &att);
                    float yaw_tolerance = wpm.accept_range_yaw;
                    //compare current yaw
                    if (att.yaw - yaw_tolerance >= 0.0f && att.yaw + yaw_tolerance < 2.f*M_PI)
                    {
                        if (att.yaw - yaw_tolerance <= wp->param4 && att.yaw + yaw_tolerance >= wp->param4)
                            wpm.yaw_reached = true;
                    }
                    else if(att.yaw - yaw_tolerance < 0.0f)
                    {
                        float lowerBound = 360.0f + att.yaw - yaw_tolerance;
                        if (lowerBound < wp->param4 || wp->param4 < att.yaw + yaw_tolerance)
                            wpm.yaw_reached = true;
                    }
                    else
                    {
                        float upperBound = att.yaw + yaw_tolerance - 2.f*M_PI;
                        if (att.yaw - yaw_tolerance < wp->param4 || wp->param4 < upperBound)
                            wpm.yaw_reached = true;
                    }
                }
            }
            break;
        }

		case MAVLINK_MSG_ID_LOCAL_POSITION:
        {
            if(msg->sysid == mavlink_system.sysid && wpm.current_active_wp_id < wpm.size)
            {
                mavlink_waypoint_t *wp = &(wpm.waypoints[wpm.current_active_wp_id]);

                if(wp->frame == MAV_FRAME_LOCAL_ENU || MAV_FRAME_LOCAL_NED)
                {
                    mavlink_local_position_t pos;
                    mavlink_msg_local_position_decode(msg, &pos);
                    //if (debug) printf("Received new position: x: %f | y: %f | z: %f\n", pos.x, pos.y, pos.z);

                    wpm.pos_reached = false;

                    // compare current position (given in message) with current waypoint
                    float orbit = wp->param1;

                    float dist;
                    if (wp->param2 == 0)
                    {
						// FIXME segment distance
                        //dist = mavlink_wpm_distance_to_segment(current_active_wp_id, pos.x, pos.y, pos.z);
                    }
                    else
                    {
                        dist = mavlink_wpm_distance_to_point(wpm.current_active_wp_id, pos.x, pos.y, pos.z);
                    }

                    if (dist >= 0.f && dist <= orbit && wpm.yaw_reached)
                    {
                        wpm.pos_reached = true;
                    }
                }
            }
            break;
        }

//		case MAVLINK_MSG_ID_CMD: // special action from ground station
//        {
//            mavlink_cmd_t action;
//            mavlink_msg_cmd_decode(msg, &action);
//            if(action.target == mavlink_system.sysid)
//            {
//                if (verbose) std::cerr << "Waypoint: received message with action " << action.action << std::endl;
//                switch (action.action)
//                {
//						//				case MAV_ACTION_LAUNCH:
//						//					if (verbose) std::cerr << "Launch received" << std::endl;
//						//					current_active_wp_id = 0;
//						//					if (wpm.size>0)
//						//					{
//						//						setActive(waypoints[current_active_wp_id]);
//						//					}
//						//					else
//						//						if (verbose) std::cerr << "No launch, waypointList empty" << std::endl;
//						//					break;
//
//						//				case MAV_ACTION_CONTINUE:
//						//					if (verbose) std::c
//						//					err << "Continue received" << std::endl;
//						//					idle = false;
//						//					setActive(waypoints[current_active_wp_id]);
//						//					break;
//
//						//				case MAV_ACTION_HALT:
//						//					if (verbose) std::cerr << "Halt received" << std::endl;
//						//					idle = true;
//						//					break;
//
//						//				default:
//						//					if (verbose) std::cerr << "Unknown action received with id " << action.action << ", no action taken" << std::endl;
//						//					break;
//                }
//            }
//            break;
//        }

		case MAVLINK_MSG_ID_WAYPOINT_ACK:
        {
            mavlink_waypoint_ack_t wpa;
            mavlink_msg_waypoint_ack_decode(msg, &wpa);

            if((msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && (wpa.target_system == mavlink_system.sysid && wpa.target_component == mavlink_system.compid))
            {
                wpm.timestamp_lastaction = now;

                if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST || wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)
                {
                    if (wpm.current_wp_id == wpm.size-1)
                    {
                        if (verbose) printf("Received Ack after having sent last waypoint, going to state MAVLINK_WPM_STATE_IDLE\n");
                        wpm.current_state = MAVLINK_WPM_STATE_IDLE;
                        wpm.current_wp_id = 0;
                    }
                }
            }
			else
			{
				if (verbose) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
			}
            break;
        }

		case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
        {
            mavlink_waypoint_set_current_t wpc;
            mavlink_msg_waypoint_set_current_decode(msg, &wpc);

            if(wpc.target_system == mavlink_system.sysid && wpc.target_component == mavlink_system.compid)
            {
                wpm.timestamp_lastaction = now;

                if (wpm.current_state == MAVLINK_WPM_STATE_IDLE)
                {
                    if (wpc.seq < wpm.size)
                    {
                        if (verbose) printf("Received MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT\n");
                        wpm.current_active_wp_id = wpc.seq;
                        uint32_t i;
                        for(i = 0; i < wpm.size; i++)
                        {
                            if (i == wpm.current_active_wp_id)
                            {
                                wpm.waypoints[i].current = true;
                            }
                            else
                            {
                                wpm.waypoints[i].current = false;
                            }
                        }
                        if (verbose) printf("New current waypoint %u\n", wpm.current_active_wp_id);
                        wpm.yaw_reached = false;
                        wpm.pos_reached = false;
                        mavlink_wpm_send_waypoint_current(wpm.current_active_wp_id);
                        mavlink_wpm_send_setpoint(wpm.current_active_wp_id);
                        wpm.timestamp_firstinside_orbit = 0;
                    }
                    else
                    {
                        if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT: Index out of bounds\n");
                    }
                }
				else
				{
					if (verbose) printf("IGNORED WAYPOINT COMMAND BECAUSE NOT IN IDLE STATE\n");
				}
            }
			else
			{
				if (verbose) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
			}
            break;
        }

		case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
        {
            mavlink_waypoint_request_list_t wprl;
            mavlink_msg_waypoint_request_list_decode(msg, &wprl);
            if(wprl.target_system == mavlink_system.sysid && wprl.target_component == mavlink_system.compid)
            {
                wpm.timestamp_lastaction = now;

                if (wpm.current_state == MAVLINK_WPM_STATE_IDLE || wpm.current_state == MAVLINK_WPM_STATE_SENDLIST)
                {
                    if (wpm.size > 0)
                    {
                        if (verbose && wpm.current_state == MAVLINK_WPM_STATE_IDLE) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u changing state to MAVLINK_WPM_STATE_SENDLIST\n", msg->sysid);
                        if (verbose && wpm.current_state == MAVLINK_WPM_STATE_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST again from %u staying in state MAVLINK_WPM_STATE_SENDLIST\n", msg->sysid);
                        wpm.current_state = MAVLINK_WPM_STATE_SENDLIST;
                        wpm.current_wp_id = 0;
                        wpm.current_partner_sysid = msg->sysid;
                        wpm.current_partner_compid = msg->compid;
                    }
                    else
                    {
                        if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
                    }
                    wpm.current_count = wpm.size;
                    mavlink_wpm_send_waypoint_count(msg->sysid,msg->compid, wpm.current_count);
                }
                else
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST because i'm doing something else already (state=%i).\n", wpm.current_state);
                }
            }
			else
			{
				if (verbose) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT MISMATCH\n");
			}

            break;
        }

		case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
        {
            mavlink_waypoint_request_t wpr;
            mavlink_msg_waypoint_request_decode(msg, &wpr);
            if(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid && wpr.target_system == mavlink_system.sysid && wpr.target_component == mavlink_system.compid)
            {
                wpm.timestamp_lastaction = now;

                //ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
                if ((wpm.current_state == MAVLINK_WPM_STATE_SENDLIST && wpr.seq == 0) || (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && (wpr.seq == wpm.current_wp_id || wpr.seq == wpm.current_wp_id + 1) && wpr.seq < wpm.size))
                {
                    if (verbose && wpm.current_state == MAVLINK_WPM_STATE_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u changing state to MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
                    if (verbose && wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm.current_wp_id + 1) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
                    if (verbose && wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm.current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u (again) from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);

                    wpm.current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;
                    wpm.current_wp_id = wpr.seq;
                    mavlink_wpm_send_waypoint(wpm.current_partner_sysid, wpm.current_partner_compid, wpr.seq);
                }
                else
                {
                    if (verbose)
                    {
                        if (!(wpm.current_state == MAVLINK_WPM_STATE_SENDLIST || wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because i'm doing something else already (state=%i).\n", wpm.current_state); break; }
                        else if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST)
                        {
                            if (wpr.seq != 0) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);
                        }
                        else if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)
                        {
                            if (wpr.seq != wpm.current_wp_id && wpr.seq != wpm.current_wp_id + 1) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, wpm.current_wp_id, wpm.current_wp_id+1);
                            else if (wpr.seq >= wpm.size) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);
                        }
                        else printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST - FIXME: missed error description\n");
                    }
                }
            }
            else
            {
                //we we're target but already communicating with someone else
                if((wpr.target_system == mavlink_system.sysid && wpr.target_component == mavlink_system.compid) && !(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid))
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, wpm.current_partner_sysid);
                }
				else
				{
					if (verbose) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
				}

            }
            break;
        }

		case MAVLINK_MSG_ID_WAYPOINT_COUNT:
			break;

		case MAVLINK_MSG_ID_WAYPOINT:
        {
            mavlink_waypoint_t wp;
            mavlink_msg_waypoint_decode(msg, &wp);

			if (verbose) printf("GOT WAYPOINT!");

            if((msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && (wp.target_system == mavlink_system.sysid && wp.target_component == mavlink_system.compid))
            {
                wpm.timestamp_lastaction = now;

                //ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
                if ((wpm.current_state == MAVLINK_WPM_STATE_GETLIST && wp.seq == 0) || (wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq == wpm.current_wp_id && wp.seq < wpm.current_count))
                {
                    if (verbose && wpm.current_state == MAVLINK_WPM_STATE_GETLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u changing state to MAVLINK_WPM_STATE_GETLIST_GETWPS\n", wp.seq, msg->sysid);
                    if (verbose && wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq == wpm.current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u\n", wp.seq, msg->sysid);
                    if (verbose && wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq-1 == wpm.current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u (again) from %u\n", wp.seq, msg->sysid);

                    wpm.current_state = MAVLINK_WPM_STATE_GETLIST_GETWPS;
                    mavlink_waypoint_t* newwp = &(wpm.rcv_waypoints[wp.seq]);
                    memcpy(newwp, &wp, sizeof(mavlink_waypoint_t));

					                    wpm.current_wp_id = wp.seq + 1;

                    if (verbose) printf ("Added new waypoint to list. X= %f\t Y= %f\t Z= %f\t Yaw= %f\n", newwp->x, newwp->y, newwp->z, newwp->param4);

                    if(wpm.current_wp_id == wpm.current_count && wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS)
                    {
                        if (verbose) printf("Got all %u waypoints, changing state to MAVLINK_WPM_STATE_IDLE\n", wpm.current_count);

                        mavlink_wpm_send_waypoint_ack(wpm.current_partner_sysid, wpm.current_partner_compid, 0);

                        if (wpm.current_active_wp_id > wpm.rcv_size-1)
                        {
                            wpm.current_active_wp_id = wpm.rcv_size-1;
                        }

                        // switch the waypoints list
						// FIXME CHECK!!!
						for (int i = 0; i < wpm.current_count; ++i)
						{
							wpm.waypoints[i] = wpm.rcv_waypoints[i];
						}
						wpm.size = wpm.current_count;

                        //get the new current waypoint
                        uint32_t i;
                        for(i = 0; i < wpm.size; i++)
                        {
                            if (wpm.waypoints[i].current == 1)
                            {
                                wpm.current_active_wp_id = i;
                                //if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
                                wpm.yaw_reached = false;
                                wpm.pos_reached = false;
								mavlink_wpm_send_waypoint_current(wpm.current_active_wp_id);
                                mavlink_wpm_send_setpoint(wpm.current_active_wp_id);
								wpm.timestamp_firstinside_orbit = 0;
                                break;
                            }
                        }

                        if (i == wpm.size)
                        {
                            wpm.current_active_wp_id = -1;
                            wpm.yaw_reached = false;
                            wpm.pos_reached = false;
                            wpm.timestamp_firstinside_orbit = 0;
                        }

                        wpm.current_state = MAVLINK_WPM_STATE_IDLE;
                    }
                    else
                    {
                        mavlink_wpm_send_waypoint_request(wpm.current_partner_sysid, wpm.current_partner_compid, wpm.current_wp_id);
                    }
                }
                else
                {
                    if (wpm.current_state == MAVLINK_WPM_STATE_IDLE)
                    {
                        //we're done receiving waypoints, answer with ack.
                        mavlink_wpm_send_waypoint_ack(wpm.current_partner_sysid, wpm.current_partner_compid, 0);
                        printf("Received MAVLINK_MSG_ID_WAYPOINT while state=MAVLINK_WPM_STATE_IDLE, answered with WAYPOINT_ACK.\n");
                    }
                    if (verbose)
                    {
                        if (!(wpm.current_state == MAVLINK_WPM_STATE_GETLIST || wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u because i'm doing something else already (state=%i).\n", wp.seq, wpm.current_state); break; }
                        else if (wpm.current_state == MAVLINK_WPM_STATE_GETLIST)
                        {
                            if(!(wp.seq == 0)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the first waypoint ID (%u) was not 0.\n", wp.seq);
                            else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
                        }
                        else if (wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS)
                        {
                            if (!(wp.seq == wpm.current_wp_id)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was not the expected %u.\n", wp.seq, wpm.current_wp_id);
                            else if (!(wp.seq < wpm.current_count)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was out of bounds.\n", wp.seq);
                            else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
                        }
                        else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
                    }
                }
            }
            else
            {
                //we we're target but already communicating with someone else
                if((wp.target_system == mavlink_system.sysid && wp.target_component == mavlink_system.compid) && !(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && wpm.current_state != MAVLINK_WPM_STATE_IDLE)
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, wpm.current_partner_sysid);
                }
                else if(wp.target_system == mavlink_system.sysid && wp.target_component == mavlink_system.compid)
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
                }
            }
            break;
        }

		case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
        {
            mavlink_waypoint_clear_all_t wpca;
            mavlink_msg_waypoint_clear_all_decode(msg, &wpca);

            if(wpca.target_system == mavlink_system.sysid && wpca.target_component == mavlink_system.compid && wpm.current_state == MAVLINK_WPM_STATE_IDLE)
            {
                wpm.timestamp_lastaction = now;

                if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
                // Delete all waypoints
				wpm.size = 0;
                wpm.current_active_wp_id = -1;
                wpm.yaw_reached = false;
                wpm.pos_reached = false;
            }
            else if (wpca.target_system == mavlink_system.sysid && wpca.target_component == mavlink_system.compid && wpm.current_state != MAVLINK_WPM_STATE_IDLE)
            {
                if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, wpm.current_state);
            }
            break;
        }

    }


}

 */



