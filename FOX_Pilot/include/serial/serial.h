/*
 * serial.h
 *
 *  Created on: 19/dic/2010
 *      Author: vito
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>

#include <defines.h>
#include <config.h>

#include <serial/ring.h>

static inline int init_serial(const char *devicename, int baud_rate, int data_bits, int stop_bits, int parity, struct termios *oldtio) {
	int BAUD;                      // derived baud rate from command line
	int DATABITS;
	int STOPBITS;
	int PARITYON;
	int PARITY;

	int fd;

	//struct sigaction saio;               //definition of signal action

	struct termios newtio;       //place for old and new port settings for serial port

	switch (baud_rate) {
	case 38400:
	default:
		BAUD = B38400;
		break;
	case 19200:
		BAUD  = B19200;
		break;
	case 9600:
		BAUD  = B9600;
		break;
	case 4800:
		BAUD  = B4800;
		break;
	case 2400:
		BAUD  = B2400;
		break;
	}  //end of switch baud_rate

	switch (data_bits) {
	case 8:
	default:
		DATABITS = CS8;
		break;
	case 7:
		DATABITS = CS7;
		break;
	case 6:
		DATABITS = CS6;
		break;
	case 5:
		DATABITS = CS5;
		break;
	}  //end of switch data_bits

	switch (stop_bits) {
	case 1:
	default:
		STOPBITS = 0;
		break;
	case 2:
		STOPBITS = CSTOPB;
		break;
	}  //end of switch stop bits

	switch (parity) {
	case 0:
	default:                       //none
		PARITYON = 0;
		PARITY = 0;
		break;
	case 1:                        //odd
		PARITYON = PARENB;
		PARITY = PARODD;
		break;
	case 2:                        //even
		PARITYON = PARENB;
		PARITY = 0;
		break;
	}  //end of switch parity

	fd = open(devicename, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		return(fd);
	}

	fcntl(fd, F_SETFL, 0);

	if (oldtio != NULL) {
		tcgetattr(fd, oldtio); // save current port settings
	}

	memset(&newtio, 0, sizeof(newtio));
//	cfsetispeed(&newtio, BAUD);
//	cfsetospeed(&newtio, BAUD);

	// set new port settings for canonical input processing
	newtio.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;  //IGNPAR
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;       //ICANON;
	newtio.c_cc[VMIN]=1;      //blocking read until 1 character arrives (meglio 10?)
	newtio.c_cc[VTIME]=0;     //inter-character timer unused
	tcflush(fd, TCIFLUSH);

	/* set all of the options */
	tcsetattr(fd, TCSANOW, &newtio);

	return(fd);
}

static inline void deinit_serial(int fd, struct termios *oldtio) {
	if (oldtio != NULL) {
		tcsetattr(fd, TCSANOW, oldtio);
	}

	close(fd);        //close the com port
}

#endif /* SERIAL_H_ */
