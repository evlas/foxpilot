/*
 * pololu.h
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

/*
 * Servo ports: 8
 * Resolution: 0.5 탎 (about 0.05�)
 * Range: 250-2750 탎
 * Logic supply voltage: 5-16 V
 * Data voltage: 0 and 5 V
 * Pulse rate: 50 Hz
 * Serial baud rate: 1200-38400 (automatically detected)
 * Current consumption: 5 mA (average)
 */

#ifndef POLOLU_H_
#define POLOLU_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <termios.h>
#include <sys/time.h>

#ifdef _SSC32_
#define POLOLU_NCHAN 32

#define POLOLU_MIN 1800
#define POLOLU_MAX 4200
#endif

/* SSC03A
 *
 * Servo ports: 8
 * Resolution: 0.5 탎 (about 0.05�)
 * Range: 250-2750 탎 (500-5500)
 * Logic supply voltage: 5-16 V
 * Data voltage: 0 and 5 V
 * Pulse rate: 50 Hz
 * Serial baud rate: 1200-38400 (automatically detected)
 * Current consumption: 5 mA (average)
 */
#ifdef _SSCMINI_
#define POLOLU_NCHAN 8

//#define POLOLU_MIN 500
//#define POLOLU_MAX 5500

#define POLOLU_MIN 1800		//900 us
#define POLOLU_MAX 4200		//2100 us
#endif


typedef struct __pololuchan_t {
	uint16_t  min[POLOLU_NCHAN];
	uint16_t  zero[POLOLU_NCHAN];
	uint16_t  max[POLOLU_NCHAN];
	uint16_t  value[POLOLU_NCHAN];
} pololuchan_t;

typedef struct __pololu_serial_t {
	bool connected; /* if not true, connect() must be run */
	char idstring[26];
	int  fd;
	struct termios oldtio;
	char device[256];
	int rate;
	int data_bits;
	int stop_bits;
	int parity;
} pololu_serial_t;

typedef enum {setParam, setSpeed, setPos7, setPos8, setPosA, setNeutr} cmd_t;

typedef struct __pololu_param_t {
  uint8_t reserved: 1;
  uint8_t onoff: 1;
  uint8_t direction: 1;
  uint8_t range: 5;
} pololu_param_t;

typedef struct __pololu_t {
  uint8_t startBit;
  uint8_t id;
  uint8_t command;
  uint8_t servo;
  uint8_t data1;
  uint8_t data2;
} pololu_t;

typedef struct __ssc_t {
  uint8_t startBit;
  uint8_t servo;
  uint8_t position;
} ssc_t;

pololu_serial_t pololu_serial;

//thread
void *pololu_loop(void *ptr);

//handler
void conf_pololu(char *device, int rate, int data_bits, int stop_bits, int parity);
int init_pololu(pololuchan_t *chan);
void deinit_pololu();

void read_attuatori_pololu(pololuchan_t *chan);

/* gets the ID of this ssc */
const char *getId();

//SSC MODE
void s_sendCommand(ssc_t *);

/* step to position <pos> on servo <servo> */
int s_stepTo(uint8_t servo, uint8_t pos);

//POLOLU MODE
void p_sendCommand(pololu_t *);
void p_sendCommand6(pololu_t *);

int p_command(uint8_t controller, uint8_t servo, cmd_t cmd, int data);

int p_enable(uint8_t controller, uint8_t servo);
int p_disable(uint8_t controller, uint8_t servo);
int p_forward(uint8_t controller, uint8_t servo);
int p_reverse(uint8_t controller, uint8_t servo);
int p_range(uint8_t controller, uint8_t servo, uint8_t range);

//Mangle
//setParam
int p_set_parameters(uint8_t servo, uint8_t param);
//setSpeed
int p_set_speed(uint8_t servo, uint8_t speedVal);
//setPos7
int p_position_7bit(uint8_t servo, uint8_t posValue);
//setPos8
int p_position_8bit(uint8_t servo, uint8_t posValue);
//setPosA
int p_position_absolute(uint8_t servo, int angle);
//setNeutr
int p_set_neutral(uint8_t servo, int angle);

#endif /* POLOLU_H_ */
