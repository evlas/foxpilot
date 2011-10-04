/*
 * loadconf.h
 *
 *  Created on: 28/mar/2011
 *      Author: vito
 */

#ifndef LOADCONF_H_
#define LOADCONF_H_

#include <stdio.h>
#include <stdlib.h>

#include <config.h>

#define CONFIG_FILE "config.cfg"

typedef struct {
	char arduimu_device[25];
	int  arduimu_rate;
	int  arduimu_data_bits;
	int  arduimu_stop_bits;
	int  arduimu_parity;
	char pololu_device[25];
	int  pololu_rate;
	int  pololu_data_bits;
	int  pololu_stop_bits;
	int  pololu_parity;
} load_data_struct_t;

load_data_struct_t load_data_struct;

int load_configuration(load_data_struct_t *load_data_struct);
int save_configuration(load_data_struct_t *load_data_struct);

int get_loadconf(load_data_struct_t *dst);
int set_loadconf(load_data_struct_t *src);


#endif /* LOADCONF_H_ */
