/*
 * pololu.c
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdarg.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/select.h>
#include <stdint.h>
#include <stdbool.h>

#include <serial/serial.h>
#include <attuatori.h>

#include "pololu.h"

pololu_serial_t pololu_serial;

void *pololu_loop(void *ptr) {
	int i;
	pololuchan_t pololuchan;

	init_pololu();

	for (i=0;i<POLOLU_NCHAN;i++) {
		pololuchan.max[i] = POLOLU_MAX;
		pololuchan.zero[i] = (POLOLU_MAX+POLOLU_MIN)/2;
		pololuchan.min[i] = POLOLU_MIN;
		pololuchan.value[i] = pololuchan.zero[i];
	}

	while (pololu_serial.connected) {
		read_attuatori_pololu(&pololuchan);
		for (i=0;i<POLOLU_NCHAN;i++) {
			p_position_absolute(i, pololuchan.value[i]);
		}
	}

	deinit_pololu();
}

void conf_pololu(char *device, int rate, int data_bits, int stop_bits, int parity){
	memset(&pololu_serial,0,sizeof(pololu_serial_t));
	sprintf(pololu_serial.device, "%s", device);
	pololu_serial.rate = rate;
	pololu_serial.data_bits = data_bits;
	pololu_serial.stop_bits = stop_bits;
	pololu_serial.parity = parity;
}

int init_pololu() {
	pololu_serial.connected = false;

	while((pololu_serial.fd = init_serial(pololu_serial.device,
									pololu_serial.rate,
									pololu_serial.data_bits,
									pololu_serial.stop_bits,
									pololu_serial.parity,
									&(pololu_serial.oldtio)))<0) {
		printf("Errore di connessione sulla seriale %s (POLOLU)\n",pololu_serial.device);
		sleep(5);
	};
	pololu_serial.connected = true;
	return(0);
}

void deinit_pololu() {
	deinit_serial(pololu_serial.fd, &pololu_serial.oldtio);
	pololu_serial.connected = false;
}

void read_attuatori_pololu(pololuchan_t *chan) {
	int i;
	attuatori_t attuatori;

	read_attuatori(&attuatori);

	for(i=0;i<NUMS_ATTUATORI;i++){
			if (attuatori.value[i] < 0) {
				chan->value[attuatori.id[i]] = (((chan->zero[attuatori.id[i]] - chan->min[attuatori.id[i]])*attuatori.value[i])/10000)+chan->zero[attuatori.id[i]];
			} else {
				chan->value[attuatori.id[i]] = (((chan->max[attuatori.id[i]] - chan->zero[attuatori.id[i]])*attuatori.value[i])/10000)+chan->zero[attuatori.id[i]];
			}
//		chan->value[attuatori.id[i]] = (((chan->max[attuatori.id[i]] - chan->min[attuatori.id[i]])*attuatori.value[i])/(attuatori[i].max-attuatori.min[i]))+chan->min[attuatori.id[i]];
	}
}


////////////////////////////////

const char *getId() {
  return (pololu_serial.idstring);
}

//SSC MODE
void s_sendCommand(ssc_t *ssc) {
  char buf[3];
  int ret;
  snprintf(buf,3,"%c%c%c",ssc->startBit,ssc->servo,ssc->position);
  ret = write(pololu_serial.fd,buf,3);
}

int s_stepTo(uint8_t servo, uint8_t pos) {
  ssc_t ssc;
  if(!pololu_serial.connected) {
	  return(0);
  }

  ssc.startBit=0xFF;

  if (servo == 0xFF) {
    return 0;
  }
  ssc.servo=servo;

  ssc.position=pos;
  if (pos == 0xFF) {
    ssc.position=0xFE;
  }

  s_sendCommand(&ssc);

  return(1);
}

//POLOLU MODE
/* write a message to the SSC */
void p_sendCommand(pololu_t *pololu) {
  char buf[5];
  int ret;
  snprintf(buf,5,"%c%c%c%c%c",pololu->startBit,pololu->id,pololu->command,pololu->servo,pololu->data1);
  ret = write(pololu_serial.fd,buf,5);
}

void p_sendCommand6(pololu_t *pololu) {
  char buf[6];
  int ret;
  pololu->servo += pololu->id*16;
  snprintf(buf,6,"%c%c%c%c%c%c",pololu->startBit,pololu->id,pololu->command,pololu->servo,pololu->data1,pololu->data2);
  ret = write(pololu_serial.fd,buf,6);
}

int p_command(uint8_t controller, uint8_t servo, cmd_t cmd, int data) {
  pololu_t pololu;
  pololu_param_t parametro;

  if(pololu_serial.connected == false) {
	  return(0);
  }

  pololu.startBit=0x80;
  pololu.id=controller;
  pololu.servo=servo;
  pololu.command=(uint8_t)cmd;

  parametro.reserved=0;
  parametro.onoff=0;
  parametro.direction=0;
  parametro.range=0;

  switch (cmd) {
    case setParam:
      pololu.data1=(uint8_t)data;
      if (data > 127) {
        pololu.data1=127;
      }
      pololu.data2=0;
      p_sendCommand(&pololu);
      break;
    case setSpeed:
      pololu.data1=(uint8_t)data;
      if (data > 127) {
        pololu.data1=127;
      }
      pololu.data2=0;
      p_sendCommand(&pololu);
      break;
    case setPos7:
      pololu.data1=(uint8_t)data;
      if (data > 127) {
        pololu.data1=127;
      }
      pololu.data2=0;
      p_sendCommand(&pololu);
      break;
    case setPos8:
      pololu.data1=(uint8_t)(data/128);
      pololu.data2=(uint8_t)(data%128);
      p_sendCommand6(&pololu);
      break;
    case setPosA:
      pololu.data1=(uint8_t)(data/128);
      pololu.data2=(uint8_t)(data%128);
      p_sendCommand6(&pololu);
      break;
    case setNeutr:
      pololu.data1=(uint8_t)(data/128);
      pololu.data2=(uint8_t)(data%128);
      p_sendCommand6(&pololu);
      break;
    default:
      return(0);
  }
  return(1);
}

int p_enable(uint8_t controller, uint8_t servo) {
  return (p_command(controller, servo, 0, 0x40));
}

int p_disable(uint8_t controller, uint8_t servo) {
  return (p_command(controller, servo, 0, 0x00));
}

int p_forward(uint8_t controller, uint8_t servo) {
  return (p_command(controller, servo, 0, 0x00));
}

int p_reverse(uint8_t controller, uint8_t servo) {
  return (p_command(controller, servo, 0, 0x20));
}

int p_range(uint8_t controller, uint8_t servo, uint8_t range) {
  return (p_command(controller, servo, 0, range & 0x1F));
}

//Mangle
//setParam
int p_set_parameters(uint8_t servo, uint8_t param) {
  //this function uses pololu mode command 1 to set speed
  //servo is the servo number (typically 0-7)
  //speedVal is servo speed (1=slowest, 127=fastest, 0=full)
  //set speedVal to zero to turn off speed limiting

  return(p_command((servo/8)+1, servo, setParam, param));
}

//setSpeed
int p_set_speed(uint8_t servo, uint8_t speedVal) {
  //this function uses pololu mode command 1 to set speed
  //servo is the servo number (typically 0-7)
  //speedVal is servo speed (1=slowest, 127=fastest, 0=full)
  //set speedVal to zero to turn off speed limiting

  return(p_command((servo/8)+1, servo, setSpeed, speedVal));
}

//setPos7
int p_position_7bit(uint8_t servo, uint8_t posValue) {
  //this function uses pololu mode command 2 to set position
  //servo is the servo number (typically 0-7)
  //posValue * range (set with command 0) adjusted by neutral (set with command 5)
  //determines servo position

  return(p_command((servo/8)+1, servo, setPos7, posValue));
}

//setPos8
int p_position_8bit(uint8_t servo, uint8_t posValue) {
  //this function uses pololu mode command 3 to set position
  //servo is the servo number (typically 0-7)
  //posValue * range (set with command 0) adjusted by neutral (set with command 5)
  //determines servo position

  return(p_command((servo/8)+1, servo, setPos8, posValue));
}

//setPosA
int p_position_absolute(uint8_t servo, int angle) {
  //this function uses pololu mode command 4 to set absolute position
  //servo is the servo number (typically 0-7)
  //angle is the absolute position from 500 to 5500
  printf("servo %d angolo %d\n",servo,angle);
  return(p_command((servo/8)+1, servo, setPosA, angle));
}

//setNeutr
int p_set_neutral(uint8_t servo, int angle) {
  //this function uses pololu mode command 5 to set neutral position
  //servo is the servo number (typically 0-7)
  //angle is the absolute position from 500 to 5500

  return(p_command((servo/8)+1, servo, setNeutr, angle));
}


