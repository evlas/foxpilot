/*
 * config.h
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <defines.h>

#define ONBOARD_PARAM_NAME_LENGTH 15    ///< Parameter names are transmitted with max. 15 chars over MAVLink
#define SW_VERSION 2000


/***************************************/
// Watchdog
#define PERIODO_WATCHDOG 1000000	//1 secondo

/***************************************/
// Pilota
#define PERIODO_PILOTA	 20000

/***************************************/
//Ground Control:
#define GCS_PORT         14550           /* default protocol port number UAV side*/
#define NET_BUF_SIZE     10000

#define RADIO_NCHAN      POLOLU_NCHAN

/***************************************/
// Sensori
//#define NUMS_SENSORI 2

/***************************************/
//Battery:
//#define BATTERY_EVENT 1 		// (boolean) 0 = don't read battery, 1 = read battery voltage (only if you have it wired up!)

/***************************************/
//Arduimu:
#define ARDUIMU 1			//0 disable 1 enable non usato

#define ARDUIMU_DEVICE   "/dev/ttyS1"
#define ARDUIMU_RATE     38400
#define ARDUIMU_DATABITS 8
#define ARDUIMU_STOPBITS 0
#define ARDUIMU_PARITY   0

/***************************************/
// ATTUATORI
#define _SSCMINI_

#ifdef _SSCMINI_
#define NUMS_ATTUATORI 8
#endif

#ifdef _SSC32_
#define NUMS_ATTUATORI 32
#endif

#define POLOLU 1			//0 disable 1 enable

#define POLOLU_DEVICE   "/dev/ttyS2"
#define POLOLU_RATE     38400
#define POLOLU_DATABITS 8
#define POLOLU_STOPBITS 0
#define POLOLU_PARITY   0


#endif /* CONFIG_H_ */
