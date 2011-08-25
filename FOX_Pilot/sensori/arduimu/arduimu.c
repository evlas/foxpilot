/*
 * arduimu.c
 *
 *  Created on: 30/05/2011
 *      Author: Vito Ammirata
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <termios.h>

#include <defines.h>
#include <config.h>

#include "arduimu.h"

#include <serial/serial.h>
#include <serial/ring.h>
#include <sensori.h>
#include <watchdog.h>

arduimu_serial_t arduimu_serial;

void *arduimu_loop(void *ptr) {
	uint8_t buffer[IMU_BUFFER_DIM];
	uint8_t ringbuffer[IMU_BUFFER_DIM];
	int res, ringcount=0;
	//int res, i, ringcount=0;
	//long delta = 0, loop_d = 0;
	//struct timeval tv1, tv2;

	arduimu_t arduimu;
	arduimu_perf_t arduimu_perf;
	arduimu_status_t arduimu_status;

	ring_buffer_t ring;

	init_ring_buffer(&ring);
	init_arduimu();

	gettimeofday(&(arduimu_status.IMU_timer),NULL);
	gettimeofday(&(arduimu_status.GPS_timer),NULL);

	memset(ringbuffer, 0, sizeof(uint8_t)*IMU_BUFFER_DIM);

	// loop while waiting for input. normally we would do something useful here
	while (arduimu_serial.connected) {
		//		gettimeofday(&tv1, NULL);
		memset(buffer, 0, sizeof(uint8_t)*IMU_BUFFER_DIM);

		if ((res = read(arduimu_serial.fd, buffer, IMU_BUFFER_DIM))>0) {
			write_ring_buffer(&ring, buffer, res);

			//			if (read_ring_buffer(&ring, &ringbuffer[ringcount], IMU_BUFFER_DIM-ringcount)>0) {
			if (read_ring_buffer(&ring, &ringbuffer[ringcount], res)>0) {
				if((ringcount = decode_arduimu(ringbuffer, IMU_BUFFER_DIM, &(arduimu), &(arduimu_perf) ,&(arduimu_status)))>0) {
					if (ringcount<(IMU_BUFFER_DIM-1)){
						memmove(ringbuffer,&ringbuffer[ringcount],IMU_BUFFER_DIM-ringcount-1);
						ringcount = IMU_BUFFER_DIM-ringcount;
					} else {
						ringcount = 0;
					}
					write_arduimu_sensori(&arduimu, &arduimu_perf, &arduimu_status);
				}
			}
		}
	}
	deinit_arduimu();
}

//MANGLE
//CONF
void conf_arduimu(char *device, int rate, int data_bits, int stop_bits, int parity){
	memset(&arduimu_serial,0,sizeof(arduimu_serial_t));
	sprintf(arduimu_serial.device, "%s", device);
	arduimu_serial.rate = rate;
	arduimu_serial.data_bits = data_bits;
	arduimu_serial.stop_bits = stop_bits;
	arduimu_serial.parity = parity;
}

//INIT
int init_arduimu() {
	arduimu_serial.connected = false;

	while((arduimu_serial.fd = init_serial(arduimu_serial.device,
			arduimu_serial.rate,
			arduimu_serial.data_bits,
			arduimu_serial.stop_bits,
			arduimu_serial.parity,
			&(arduimu_serial.oldtio)))<0) {
		printf("Errore di connessione sulla seriale %s (ARDUIMU)\n",arduimu_serial.device);
		sleep(5);
	};

	arduimu_serial.connected = true;
	return(0);
}

//CLOSE
void deinit_arduimu(){
	deinit_serial(arduimu_serial.fd, &arduimu_serial.oldtio);
	arduimu_serial.connected = false;
}

void write_arduimu_sensori(arduimu_t *arduimu, arduimu_perf_t *arduimu_perf, arduimu_status_t *arduimu_status) {
	imu_t imu;
	gps_t gps;

	read_imu_sensori(&imu);

	imu.pitch = arduimu->pitch;
	imu.roll = arduimu->roll;
	imu.yaw = arduimu->yaw;

	write_imu_sensori(&imu);

	read_gps_sensori(&gps);

	gps.GPS_fix = arduimu->GPS_fix;
	gps.altitude = arduimu->altitude * 10; // da decimetri a centimetri
	gps.latitude = arduimu->latitude;
	gps.longitude = arduimu->longitude;
	gps.speed = arduimu->gps_speed;

	write_gps_sensori(&gps);
}

//DECODE
int decode_arduimu(uint8_t *buf, int len, arduimu_t *imu, arduimu_perf_t *perf, arduimu_status_t *status) {
	int i = 0;
	int IMU_step = 0;
	int message_num = 0, payload_length = 0, payload_counter = 0;
	struct timeval tv;
	uint8_t ck_a = 0, ck_b = 0, IMU_ck_a = 0, IMU_ck_b = 0,IMU_buffer[IMU_BUFFER_DIM];

	if (len > 0) {
		for (i=0; i<len; i++) {	// Process bytes received
			switch(IMU_step) {	 	//Normally we start from zero. This is a state machine
			case 0:
				if(buf[i] == 0x44) {
					IMU_step++;      //First byte of data packet header is correct, so jump to the next step
				}
				break;

			case 1:
				if(buf[i] == 0x49) {
					IMU_step++;	     //Second byte of data packet header is correct
				} else {
					IMU_step=0;		 //Second byte is not correct so restart to step zero and try again.
				}
				break;

			case 2:
				if(buf[i] == 0x59) {
					IMU_step++;	     //Third byte of data packet header is correct
				} else {
					IMU_step=0;		 //Third byte is not correct so restart to step zero and try again.
				}
				break;

			case 3:
				if(buf[i] == 0x64) {
					IMU_step++;	//Fourth byte of data packet header is correct, Header complete
				} else {
					IMU_step=0;		 //Fourth byte is not correct so restart to step zero and try again.
				}
				break;

			case 4:
				payload_length = buf[i];
				if (payload_length > (len-i-1)) { //da verificare
					return(i-4);
				}
				checksum_check(&ck_a, &ck_b, payload_length);
				IMU_step++;
				if (payload_length > 28) {
					IMU_step=0;	 //Bad data, so restart to step zero and try again.
					payload_counter=0;
					ck_a = 0;
					ck_b = 0;
					status->imu_payload_error_count++;
				}
				break;

			case 5:
				message_num = buf[i];
				checksum_check(&ck_a, &ck_b, buf[i]);
				IMU_step++;
				break;

			case 6:	// Payload data read...
				// We stay in this state until we reach the payload_length
				IMU_buffer[payload_counter] = buf[i];
				checksum_check(&ck_a, &ck_b, buf[i]);
				payload_counter++;
				if (payload_counter >= payload_length) {
					IMU_step++;
				}
				break;

			case 7:
				IMU_ck_a=buf[i];	 // First checksum byte
				IMU_step++;
				break;

			case 8:
				IMU_ck_b=buf[i];	 // Second checksum byte

				// We end the IMU/GPS read...
				// Verify the received checksum with the generated checksum..
				if((ck_a == IMU_ck_a) && (ck_b == IMU_ck_b)) {
					if (message_num == 0x02) {
						IMU_join_data(imu, IMU_buffer, status);
						gettimeofday(&(status->IMU_timer),NULL);
					} else if (message_num == 0x03) {
						GPS_join_data(imu, IMU_buffer, status);
						gettimeofday(&(status->GPS_timer),NULL);
					} else if (message_num == 0x04) {
						IMU2_join_data(imu, IMU_buffer, status);
						gettimeofday(&(status->IMU_timer),NULL);
					} else if (message_num == 0x0a) {
						PERF_join_data(perf, IMU_buffer);
					} else {
						status->imu_invalid_msg_count++;
					}
				} else {
					status->imu_checksum_error_count++;
				}
				// Variable initialization
				IMU_step = 0;
				payload_counter = 0;
				ck_a = 0;
				ck_b = 0;
				gettimeofday(&(status->IMU_timer),NULL); //Restarting timer...
				break;
			}
		}// End for...
	}

	gettimeofday(&tv,NULL);

	if(((tv.tv_sec - status->IMU_timer.tv_sec)*(10^6) +(tv.tv_usec - status->IMU_timer.tv_usec)) > 500000) {	//If we don't receive IMU data in 1/2 second, set flag
		imu->IMU_ok = false;
	}

	if(((tv.tv_sec - status->IMU_timer.tv_sec)*(10^6) +(tv.tv_usec - status->IMU_timer.tv_usec)) > 2000000) {
		imu->GPS_fix = false;
	}
	return(i);
}

/****************************************************************
 * SUB MANGLE
 ****************************************************************/
void IMU_join_data(arduimu_t *imu, uint8_t *buffer, arduimu_status_t *status) {
	status->imu_messages_received++;

	//Storing IMU roll
	memcpy(&(imu->roll), buffer, 2);
	switch16(imu->roll);

	//Storing IMU pitch
	memcpy(&(imu->pitch), &buffer[2], 2);
	switch16(imu->pitch);

	//Storing IMU heading (yaw)
	memcpy(&(imu->yaw), &buffer[4], 2);
	switch16(imu->yaw);

	imu->air_speed=-1;

	imu->IMU_ok = true;
}

void IMU2_join_data(arduimu_t *imu, uint8_t *buffer, arduimu_status_t *status) {
	status->imu_messages_received++;

	//Storing IMU roll
	memcpy(&(imu->roll), buffer, 2);
	switch16(imu->roll);

	//Storing IMU pitch
	memcpy(&(imu->pitch), &buffer[2], 2);
	switch16(imu->pitch);

	//Storing IMU heading (yaw)
	memcpy(&(imu->yaw), &buffer[4], 2);
	switch16(imu->yaw);
	//Storing IMU heading (yaw)

	//Storing airspeed
	memcpy(&(imu->air_speed), &buffer[6], 2);
	switch16(imu->air_speed);

	imu->IMU_ok = true;
}

/****************************************************************
 *
 ****************************************************************/
void GPS_join_data(arduimu_t *imu, uint8_t *buffer, arduimu_status_t *status) {
	status->gps_messages_received++;

	//Storing GPS longitude
	memcpy(&(imu->longitude), buffer, 4);
	switch32(imu->longitude);

	//Storing GPS latitude
	memcpy(&(imu->latitude), &buffer[4], 4);
	switch32(imu->latitude);

	//Storing GPS altitude above the sea level
	memcpy(&(imu->altitude), &buffer[8], 2);
	switch16(imu->altitude);

	//Storing Speed (3-D)
	memcpy(&(imu->gps_speed), &buffer[10], 2);
	switch16(imu->gps_speed);

	//Storing course
	memcpy(&(imu->gps_course), &buffer[12], 2);
	switch16(imu->gps_course);

	//We skip the gps ground course because we use yaw value from the IMU for ground course
	//j += 2;void checksum_check(uint8_t *ck_a, uint8_t *ck_b, uint8_t data)
	//iTOW = join_4_bytes(&IMU_buffer[j]);		//	Interval Time of the Week in milliseconds
	//j +=4;
	//imu_health = IMU_buffer[j++];

	//GPS_new_data = true;
	imu->GPS_fix = true;
}

/****************************************************************
 *
 ****************************************************************/
void PERF_join_data(arduimu_perf_t *imu, uint8_t *buffer) {

	//PERF time in milliseconds of reporting interval
	memcpy(&(imu->perf_mon_timer), buffer, 4);
	switch32(imu->perf_mon_timer);

	//IMU main loop cycles in reporting interval
	memcpy(&(imu->mainLoop_count), &buffer[4], 2);
	switch32(imu->mainLoop_count);

	//Max IMU main loop time in milliseconds
	memcpy(&(imu->G_Dt_max), &buffer[6], 2);
	switch32(imu->G_Dt_max);

	imu->gyro_sat_count = buffer[8];
	imu->adc_constraints = buffer[9];
	imu->renorm_sqrt_count = buffer[10];
	imu->renorm_blowup_count = buffer[11];
	imu->gps_payload_error_count = buffer[12];
	imu->gps_checksum_error_count = buffer[13];
	imu->gps_pos_fix_count = buffer[14];
	imu->gps_nav_fix_count = buffer[15];
	imu->gps_messages_sent = buffer[16];
}

void checksum_check(uint8_t *ck_a, uint8_t *ck_b, uint8_t data) {
	*ck_a += data;
	*ck_b += *ck_a;
}


