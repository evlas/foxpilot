/*
 * mybatteria.h
 *
 *  Created on: 01/06/2011
 *      Author: Vito Ammirata
 */

// ***********************************************************************************
// ******************************** BatteryMonitor  **********************************
// ***********************************************************************************
/* Circuit:

  Vin--D1--R1--|--R2--GND
               |
               |
              Vout
 */
//If lipo is 12.6V and diode drop is 0.6V (res 12.0V), the voltage from divider network will be = 2.977V
//calculation: AREF/1024.0 is Vout of divider network
//Vin = lipo voltage minus the diode drop
//Vout = (Vin*R2) * (R1+R2)
//Vin = (Vout * (R1+R2))/R2
//Vin = ((((AREF/1024.0)*adDECIMAL) * (R1+R2)) / R2) + Diode drop //(aref/1024)*adDecimal is Vout
//Vout connected to Ain0

#ifndef MYBATTERIA_H_
#define MYBATTERIA_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <termios.h>
#include <sys/time.h>

#include <defines.h>
#include <config.h>

#include <sensori.h>

#define BATTERYCHANV "/sys/bus/platform/devices/at91_adc/chan0"	//FOX BOARD
#define BATTERYCHANA "/sys/bus/platform/devices/at91_adc/chan1"	//FOX BOARD

#define R1_battery  15000
#define R2_battery  7500
#define diodo_battery 900	//0.9V * 1000
#define Aref_battery  3300	//3.3V * 1000

#define Periodo_battery  1000000	//1 secondi

//#define ScaleFactor_battery (Aref_battery * ((R1_battery + R2_battery) / R2_battery))
#define ScaleLevel_battery 1024

typedef struct __mybatteria_t {
	int volt;			//milliVolt - lettura
	int volt_nom;		//milliVolt - dati di targa della batteria
	int volt_max;		//milliVolt - Lipo 4,2 * celle
	int volt_min;		//milliVolt - Lipo 3,2 * celle

	int ampora_nom;		// milliAmpereOra - dati di targa della batteria

	int corrente;		// milliAmpere - lettura
	int corrente_max;	// milliAmpere - min tra dati di targa della batteria (25C x 4500mAh) 112,5A e 90A (AttoPilot Voltage and Current Sense Breakout)

	int capacita;		// milliWatt
	int capacita_nom;	// milliWatt

	uint16_t stato; //0->0% 10000->100%
} mybatteria_t;

typedef struct __mybatteria_conf_t {
	bool connected; /* if not true, connect() must be run */
	FILE *fd_batteriav;
	FILE *fd_batteriaa;
	char devicev[256];	//BATTERYCHAN
	char devicea[256];	//BATTERYCHAN
	int  R1;
	int  R2;
	int  diodo;
	int  Aref;
	int  ScaleFactor;
	int  ScaleLevel;
	int  periodo;
} mybatteria_conf_t;

mybatteria_conf_t batteria_conf;

//thread
void *mybatteria_loop(void *ptr);

void conf_mybatteria(char *devicev, char *devicea, int R1, int R2, int diodo, int Aref, int periodo);
int init_mybatteria(mybatteria_t *oldvalue);
void deinit_mybatteria();

void write_mybatteria_sensori(mybatteria_t *batteria);

mybatteria_t  misura_mybatteria(mybatteria_t oldvalue);
int  readBatteryVoltage();
void lowBatteryEvent();
void highBatteryEvent();

#endif /* BATTERIA_H_ */

