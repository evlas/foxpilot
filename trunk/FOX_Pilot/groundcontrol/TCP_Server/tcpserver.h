/*
 * tcpserver.h
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>
#include <groundcontrol/proto.h>

fd_set master;
int fdmax;
int listener;

void *tcpserver_loop(void *);

int send_tcpserver(uint8_t *buf, int nbytes);

#endif /* TCPSERVER_H_ */
