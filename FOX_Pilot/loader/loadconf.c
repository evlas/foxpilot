/*
 * loadconf.c
 *
 *  Created on: 28/mar/2011
 *      Author: vito
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <config.h>
#include <defines.h>

//#include <groundcontrol.h>

#include "loadconf.h"

int load_configuration(load_data_struct_t *load_data_struct) {
	FILE *fp;
	load_data_struct_t loaddata;

	char tag[25], value[25];

	memset(&loaddata,0,sizeof(load_data_struct_t));

#if ARDUIMU == 1
	memcpy(loaddata.arduimu_device, ARDUIMU_DEVICE, sizeof(loaddata.arduimu_device));
	loaddata.arduimu_rate = ARDUIMU_RATE;
	loaddata.arduimu_data_bits = ARDUIMU_DATABITS;
	loaddata.arduimu_stop_bits = ARDUIMU_STOPBITS;
	loaddata.arduimu_parity = ARDUIMU_PARITY;
#endif
#if POLOLU == 1
	memcpy(loaddata.pololu_device, POLOLU_DEVICE, sizeof(loaddata.pololu_device));
	loaddata.pololu_rate = POLOLU_RATE;
	loaddata.pololu_data_bits = POLOLU_DATABITS;
	loaddata.pololu_stop_bits = POLOLU_STOPBITS;
	loaddata.pololu_parity = POLOLU_PARITY;
#endif

	if((fp = fopen(CONFIG_FILE, "r")) == NULL) {
		printf("Manca il file %s\n", CONFIG_FILE);
	} else {

		memset(&loaddata, 0, sizeof(load_data_struct_t));

		while ((fscanf(fp, "%s = %s", tag, value)) != EOF){
#if ARDUIMU == 1
			if(strcmp(tag, "arduimu_device") == 0) {
				memcpy(loaddata.arduimu_device,value,sizeof(loaddata.arduimu_device));
				printf("arduimu_device: %s\n", loaddata.arduimu_device);
			}
			if(strcmp(tag, "arduimu_rate") == 0) {
				loaddata.arduimu_rate = atoi(value);
				printf("arduimu_rate: %d\n", loaddata.arduimu_rate);
			}
			if(strcmp(tag, "arduimu_data_bits") == 0) {
				loaddata.arduimu_data_bits = atoi(value);
				printf("arduimu_data_bits: %d\n", loaddata.arduimu_data_bits);
			}
			if(strcmp(tag, "arduimu_stop_bits") == 0) {
				loaddata.arduimu_stop_bits = atoi(value);
				printf("arduimu_stop_bits: %d\n", loaddata.arduimu_stop_bits);
			}
			if(strcmp(tag, "arduimu_parity") == 0) {
				loaddata.arduimu_parity = atoi(value);
				printf("arduimu_parity: %d\n", loaddata.arduimu_parity);
			}
#endif
#if POLOLU == 1
			if(strcmp(tag, "pololu_device") == 0) {
				memcpy(loaddata.pololu_device,value,sizeof(loaddata.pololu_device));
				printf("pololu_device: %s\n", loaddata.pololu_device);
			}
			if(strcmp(tag, "pololu_rate") == 0) {
				loaddata.pololu_rate = atoi(value);
				printf("pololu_rate: %d\n", loaddata.pololu_rate);
			}
			if(strcmp(tag, "pololu_data_bits") == 0) {
				loaddata.pololu_data_bits = atoi(value);
				printf("pololu_data_bits: %d\n", loaddata.pololu_data_bits);
			}
			if(strcmp(tag, "pololu_stop_bits") == 0) {
				loaddata.pololu_stop_bits = atoi(value);
				printf("pololu_stop_bits: %d\n", loaddata.pololu_stop_bits);
			}
			if(strcmp(tag, "pololu_parity") == 0) {
				loaddata.pololu_parity = atoi(value);
				printf("pololu_parity: %d\n", loaddata.pololu_parity);
			}
#endif
		}
		fclose(fp);
	}
	memcpy(load_data_struct,&loaddata, sizeof(load_data_struct_t));

	return(0);
}

int save_configuration(load_data_struct_t *load_data_struct) {
	FILE *fp;

	if((fp = fopen(CONFIG_FILE, "w")) == NULL) {
		printf("Errore nel creare il file %s\n", CONFIG_FILE);
		exit(errno);
	}

#if ARDUIMU == 1
	fprintf(fp, "arduimu_device = %.25s\n", load_data_struct->arduimu_device);
	fprintf(fp, "arduimu_rate = %d\n", load_data_struct->arduimu_rate);
	fprintf(fp, "arduimu_data_bits = %d\n", load_data_struct->arduimu_data_bits);
	fprintf(fp, "arduimu_stop_bits = %d\n", load_data_struct->arduimu_stop_bits);
	fprintf(fp, "arduimu_parity = %d\n", load_data_struct->arduimu_parity);
#endif
#if POLOLU == 1
	fprintf(fp, "pololu_device = %.25s\n", load_data_struct->pololu_device);
	fprintf(fp, "pololu_rate = %d\n", load_data_struct->pololu_rate);
	fprintf(fp, "pololu_data_bits = %d\n", load_data_struct->pololu_data_bits);
	fprintf(fp, "pololu_stop_bits = %d\n", load_data_struct->pololu_stop_bits);
	fprintf(fp, "pololu_parity = %d\n", load_data_struct->pololu_parity);
#endif

	fclose(fp);

	return(0);
}

int get_loadconf(load_data_struct_t *a) {
	memcpy(a,&load_data_struct,sizeof(load_data_struct_t));
	return(1);
}

int set_loadconf(load_data_struct_t *a) {
	memcpy(&load_data_struct,a,sizeof(load_data_struct_t));
	return(save_configuration(a));
}
