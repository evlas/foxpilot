/*
 * tcpserver.c
 *
 *  Created on: 21/ott/2010
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

#include <defines.h>
#include <config.h>
#include "tcpserver.h"
#include <groundcontrol.h>
#include <groundcontrol/proto.h>
#include <watchdog.h>

fd_set master;
int fdmax;
int listener;

//thread TCPServer
void *tcpserver_loop (void *ptr) {
	int count = 0, ret_sel=0;
	/* master file descriptor list */
	//fd_set master;
	/* temp file descriptor list for select() */
	fd_set read_fds;

	struct sockaddr_in serveraddr;
	struct sockaddr_in clientaddr;

	/* maximum file descriptor number */
	//int fdmax;
	/* listening socket descriptor */
	//int listener;
	/* newly accept()ed socket descriptor */
	int newfd;
	/* buffer for client data */
	uint8_t buf[NET_BUF_SIZE];
	int nbytes;
	/* for setsockopt() SO_REUSEADDR, below */
	int yes = 1;
	int addrlen;

	int i, j;

	/* clear the master and temp sets */
	FD_ZERO(&master);
	FD_ZERO(&read_fds);

	/* get the listener */
	if((listener = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		perror("Server-socket() error lol!");
		/*just exit lol!*/
		exit(1);
	}
	//  printf("Server-socket() is OK...\n");

	/*"address already in use" error message */
	if(setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
		perror("Server-setsockopt() error lol!");
		exit(1);
	}
	//  printf("Server-setsockopt() is OK...\n");

	/* bind */
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = INADDR_ANY;
	serveraddr.sin_port = htons(GCS_PORT);
	memset(&(serveraddr.sin_zero), 0, 8);

	if(bind(listener, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) == -1) {
		perror("Server-bind() error lol!");
		exit(1);
	}
	//  printf("Server-bind() is OK...\n");

	/* listen */
	if(listen(listener, 10) == -1) {
		perror("Server-listen() error lol!");
		exit(1);
	}
	//  printf("Server-listen() is OK...\n");

	/* add the listener to the master set */
	FD_SET(listener, &master);
	/* keep track of the biggest file descriptor */
	fdmax = listener; /* so far, it's this one*/

	/* loop */
	while(1) {
		memset(buf,0,NET_BUF_SIZE);

		/* copy it */
		read_fds = master;

		ret_sel = select(fdmax+1, &read_fds, (fd_set *) 0, (fd_set *) 0, NULL);
		//printf("ret_sel %d\n", ret_sel);
		if(ret_sel == -1) {
			perror("Server-select() error lol!");
			exit(1);
		}
		//    printf("Server-select() is OK...\n");

		if(ret_sel > 0) {
			/*run through the existing connections looking for data to be read*/
			for(i = 0; i < fdmax+1; i++) {
				if(FD_ISSET(i, &read_fds)) {

					if(i == listener) {
						/* handle new connections */
						addrlen = sizeof(clientaddr);
						if((newfd = accept(listener, (struct sockaddr *)&clientaddr, (socklen_t *) &addrlen)) == -1) {
							if(count>1) {
								count--;
							} else {
								set_connesso_watchdog(false);
								count--;
							}
						} else {
							//            printf("Server-accept() is OK...\n");
							FD_SET(newfd, &master); /* add to master set */
							if(newfd > fdmax) { /* keep track of the maximum */
								fdmax = newfd;
							}
							set_connesso_watchdog(true);
							count++;
							//            printf("%s: New connection from %s on socket %d\n", argv[0], inet_ntoa(clientaddr.sin_addr), newfd);
						}
					} else {
						// handle data from a client
						if((nbytes = recv(i, buf, sizeof(buf), 0)) <= 0) {
							// got error or connection closed by client
							if(nbytes == 0) {
								//connection closed
								printf("socket %d hung up\n", i);
							} else {
								perror("recv() error lol!");
							}
							if(count>1) {
								count--;
							} else {
								set_connesso_watchdog(false);
								count--;
							}
							/* close it... */
							close(i);
							/* remove from master set */
							FD_CLR(i, &master);
						} else {
							set_connesso_watchdog(true);

							for(j = 0; j <= fdmax; j++) {
								if(FD_ISSET(j, &master)) {
									if(j != listener && j != i) {
										if(send(j, buf, nbytes, 0) == -1) {
											perror("send() error lol!");
										}
									}

								}

								//write_tcpserver(buf, nbytes);

								if ((nbytes = handle_message(buf, nbytes)) > 0) {
									send_tcpserver(buf, nbytes);
							    }
							}
						}
					}
				}
			}
			set_groundcontrol_watchdog(true);
		}
	}
}

int send_tcpserver(uint8_t *buf, int nbytes) {
	int j;

	//printf("buf: %s\n",buf);

	for(j = 0; j <= fdmax; j++) {
		if(FD_ISSET(j, &master)) {
//      	if(j != listener && j != i) {
			if(j != listener) {
				if(send(j, buf, nbytes, 0) == -1) {
					perror("send() error lol!");
				}
			}
		}
	}

	return(0);
}

