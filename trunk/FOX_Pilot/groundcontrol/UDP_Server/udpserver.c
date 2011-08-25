/*
 * udpserver.c
 *
 *  Created on: 06/apr/2011
 *      Author: vito
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>

#include <defines.h>
#include <config.h>
#include "udpserver.h"
#include <groundcontrol.h>
#include <groundcontrol/proto.h>
#include <watchdog.h>

int client_sk;
int dst_ip;

void *udpserver_loop (void *ptr) {
	struct sockaddr_in si_me, si_other;
	int s, i, slen=sizeof(si_other),nbytes;
	int yes = 1;
	uint8_t buf[NET_BUF_SIZE];
	client_sk = -1;
	dst_ip = inet_addr("0.0.0.0");

	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		perror("Server-socket() error lol!");
		/*just exit lol!*/
		exit(1);
	}

	if(setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
		perror("Server-setsockopt() error lol!");
		exit(1);
	}

	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(GCS_PORT+5);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me))==-1) {
		perror("Server-bind() error lol!");
		/*just exit lol!*/
		exit(1);
	}

	if ((client_sk=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		printf("Errore socket() UDP\n");
		exit(1);
	}

	while(1) {
		memset(buf,0,NET_BUF_SIZE);
		if ((nbytes=recvfrom(s, buf, NET_BUF_SIZE, 0, (struct sockaddr *)&si_other, &slen))>0) {
			set_connesso_watchdog(true);
			set_groundcontrol_watchdog(true);
			if (dst_ip != si_other.sin_addr.s_addr) {
				dst_ip = si_other.sin_addr.s_addr;
				send_mav_boot();
			}

			handle_message(buf, nbytes);
		}
	}
	set_groundcontrol_watchdog(false);
}

int send_udpserver(uint8_t *buf, int nbytes) {
	struct sockaddr_in si_other;
	int slen=sizeof(si_other);

	if (client_sk <= 0) {
		return(-1);
	}

	memset((uint8_t *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(GCS_PORT);
	si_other.sin_addr.s_addr = dst_ip;

	//	if (inet_aton(dst_ip, &si_other.sin_addr)==0) {
	//		printf("Errore inet_aton()\n");
	//		return(1);
	//	}

	return (sendto(client_sk, buf, nbytes, 0, (struct sockaddr *)&si_other, slen));
}

