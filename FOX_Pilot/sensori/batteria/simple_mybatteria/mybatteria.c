/*
 * mybatteria.c
 *
 *  Created on: 01/06/2011
 *      Author: Vito Ammirata
 */

/*
 * batteria.h
 *
 *  Created on: 19/apr/2011
 *      Author: vito
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#include <defines.h>
#include <config.h>

#include <math/range.h>
#include <groundcontrol.h>
#include <sensori.h>

#include <math/range.h>

#include "../mybatteria.h"

mybatteria_conf_t batteria_conf;

void *mybatteria_loop(void *ptr){
	mybatteria_t oldvalue;
	mybatteria_t newvalue;

	init_mybatteria(&oldvalue);

	while (batteria_conf.connected) {
		newvalue = misura_mybatteria(oldvalue);
		write_mybatteria_sensori(&newvalue);
		oldvalue = newvalue;
		usleep(batteria_conf.periodo);
	}
	deinit_mybatteria();
}

void conf_mybatteria(char *device, int R1, int R2, int diodo, int Aref, int periodo) {
	memset(&batteria_conf,0,sizeof(mybatteria_conf_t));

	sprintf(batteria_conf.devicev, "%s", device);

	batteria_conf.R1 = R1;
	batteria_conf.R2 = R2;
	batteria_conf.diodo = diodo;
	batteria_conf.Aref = Aref;
	batteria_conf.periodo = periodo;

	batteria_conf.ScaleFactor = (batteria_conf.Aref * (batteria_conf.R1 + batteria_conf.R2))/batteria_conf.R2;
	batteria_conf.ScaleLevel = ScaleLevel_battery;
}


int init_mybatteria(mybatteria_t *oldvalue) {
	memset(oldvalue, 0, sizeof(mybatteria_t));

	while((batteria_conf.fd_batteriav = fopen(batteria_conf.devicev,"r"))==NULL)  {
		printf("Impossibile leggere la batteria\n");
		batteria_conf.connected = false;
		sleep(5);
	}

	batteria_conf.connected = true;

	oldvalue->volt_nom = 11100;	//milli volt
	oldvalue->ampora_nom = 4500; //mAh

	oldvalue->capacita_nom = oldvalue->ampora_nom * oldvalue->volt_nom * 3600;
	oldvalue->capacita = oldvalue->capacita_nom;

	oldvalue->volt_max = 12600; //milli volt
	oldvalue->volt_min =  9600; //milli volt

	oldvalue->stato = 10000;

	oldvalue->volt = 0;			//milliVolt - lettura
	oldvalue->corrente = 0;		// milliAmpere - lettura

	// milliAmpere - min tra dati di targa della batteria (25C x 4500mAh) 112,5A e 89A (AttoPilot Voltage and Current Sense Breakout)
	oldvalue->corrente_max = min(89000, 20 * oldvalue->ampora_nom);

    return(0);
}

void deinit_mybatteria() {
	fclose(batteria_conf.fd_batteriav);
}

void write_mybatteria_sensori(mybatteria_t *mybatteria){
	batteria_t batter;

	read_batteria_sensori(&batter);

	batter.volt = mybatteria->volt;
	batter.stato = mybatteria->stato;

	write_batteria_sensori(&batter);
}

mybatteria_t  misura_mybatteria(mybatteria_t oldvalue) {
	int batteryVoltage, batteryCurrent;
	mybatteria_t newvalue;

	newvalue = oldvalue;

	if(oldvalue.volt == 0) {
		newvalue.volt = readBatteryVoltage();
		newvalue.stato = ((clamp(newvalue.volt,newvalue.volt_min,newvalue.volt_max) - newvalue.volt_min)*10000)/(newvalue.volt_max-newvalue.volt_min);
		return(newvalue);
	}

//	batteryVoltage = filterSmooth(readBatteryVoltage(), oldvalue.volt, 50);
	batteryVoltage = readBatteryVoltage();

/*	if(batteryVoltage < 370) {             //lipo 1s
		batteryVoltageMax = 420;
		batteryVoltageMin = 320;
	} else if (batteryVoltage < (370*2)) { //lipo 2s
		batteryVoltageMax = 420*2;
		batteryVoltageMin = 320*2;
	} else if (batteryVoltage < (370*3)) { //lipo 3s
		batteryVoltageMax = 420*3;
		batteryVoltageMin = 320*3;
	} else if (batteryVoltage < (370*4)) { //lipo 4s
		batteryVoltageMax = 420*4;
		batteryVoltageMin = 320*4;
	}
*/
	if (batteryVoltage < oldvalue.volt_min) {
		lowBatteryEvent();
	}

	newvalue.volt = batteryVoltage;
	newvalue.stato = ((clamp(newvalue.volt,newvalue.volt_min,newvalue.volt_max) - newvalue.volt_min)*10000)/(newvalue.volt_max-newvalue.volt_min);

	return(newvalue);
}

int readBatteryVoltage() {
	int n, res;
	//fread(&n, sizeof(int), 1, fd_batteria);
	res = fscanf(batteria_conf.fd_batteriav, "%d", &n);
	rewind(batteria_conf.fd_batteriav);

	return ((n * batteria_conf.ScaleFactor)/batteria_conf.ScaleLevel + batteria_conf.diodo);
}

void lowBatteryEvent() {
	set_failsafe_groundcontrol(true);
}

void highBatteryEvent() {
	set_failsafe_groundcontrol(true);
}


