/*
 * udpserver.h
 *
 *  Created on: 06/apr/2011
 *      Author: vito
 */

#ifndef UDPSERVER_H_
#define UDPSERVER_H_

int client_sk;
int dst_ip;

void *udpserver_loop(void *);

int send_udpserver(uint8_t *buf, int nbytes);

#endif /* UDPSERVER_H_ */
