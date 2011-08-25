/*
 * arduimu.h
 *
 *  Created on: 30/05/2011
 *      Author: Vito Ammirata
 */

#ifndef ARDUIMU_H_
#define ARDUIMU_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <termios.h>
#include <sys/time.h>

#include <defines.h>
#include <config.h>

#include <sensori.h>

#define IMU_BUFFER_DIM  36      //4 char (DIYd) + 30 char (MAX DIM PAYLOAD) + 2 char checksum

//extern int imu_wait_flag;
//void imu_signal_handler_IO (int status);

/*
IMU Message format
Byte(s)		 Value
0-3		 Header "DIYd"
4						Payload length	= 6
5						Message ID = 2
6,7			roll		Integer (degrees*100)
8,9			pitch		Integer (degrees*100)
10,11		yaw			Integer (degrees*100)
12,13					checksum

IMU Message format - new with Airspeed!
Byte(s)		 Value
0-3		 Header "DIYd"
4						Payload length	= 8
5						Message ID = 4
6,7			roll		Integer (degrees*100)
8,9			pitch		Integer (degrees*100)
10,11		yaw			Integer (degrees*100)
12,13		airspeed	Integer (meters*100)
14,15					checksum


GPS Message format
Byte(s)		 Value
0-3		 Header "DIYd"
4								Payload length = 14
5								Message ID = 3
6-9			longitude			Integer (value*10**7)
10-13		latitude			Integer (value*10**7)
14,15		altitude			Integer (meters*10)
16,17		gps speed			Integer (M/S*100)
18,19		gps course			not used
20,21		checksum
*/

typedef struct __arduimu_t {
  bool IMU_ok;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t air_speed;
  bool GPS_fix;
  int32_t longitude;
  int32_t latitude;
  int16_t altitude;
  int16_t gps_speed;
  int16_t gps_course;
} arduimu_t;

typedef struct __arduimu_perf_t {
  uint32_t perf_mon_timer;		// time in milliseconds of reporting interval
  uint16_t mainLoop_count;
  float    G_Dt_max;
  uint8_t  gyro_sat_count;
  uint8_t  adc_constraints;
  uint8_t  renorm_sqrt_count;
  uint8_t  renorm_blowup_count;
  uint8_t  gps_payload_error_count;
  uint8_t  gps_checksum_error_count;
  uint8_t  gps_pos_fix_count;
  uint8_t  gps_nav_fix_count;
  uint8_t  gps_messages_sent;
} arduimu_perf_t;

typedef struct __arduimu_status_t {
  struct timeval IMU_timer; //used to set PROBLEM flag if no data is received.
  struct timeval GPS_timer;
  long imu_messages_received;
  long imu_payload_error_count;
  long imu_invalid_msg_count;
  long imu_checksum_error_count;
  long gps_messages_received;
} arduimu_status_t;

typedef struct __arduimu_serial_t {
	bool connected; /* if not true, connect() must be run */
	int fd;
	struct termios oldtio;
	char device[256];
	int rate;
	int data_bits;
	int stop_bits;
	int parity;
} arduimu_serial_t;

arduimu_serial_t arduimu_serial;

//thread
void *arduimu_loop(void *ptr);

//handler
void conf_arduimu(char *device, int rate, int data_bits, int stop_bits, int parity);
int init_arduimu();
void deinit_arduimu();

void write_arduimu_sensori(arduimu_t *arduimu, arduimu_perf_t *arduimu_perf, arduimu_status_t *arduimu_status);

int decode_arduimu(uint8_t *buf, int len, arduimu_t *imu, arduimu_perf_t *perf, arduimu_status_t *status);

void IMU_join_data(arduimu_t *imu, uint8_t *buffer, arduimu_status_t *status);
void IMU2_join_data(arduimu_t *imu, uint8_t *buffer, arduimu_status_t *status);
void GPS_join_data(arduimu_t *imu, uint8_t *buffer, arduimu_status_t *status);
void PERF_join_data(arduimu_perf_t *imu, uint8_t *buffer);
void checksum_check(uint8_t *ck_a, uint8_t *ck_b, uint8_t data);

//checksum
int  imu_checksum (uint8_t *, int);

#endif

