/*
 * watchdog.c
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>

#include <defines.h>
#include <config.h>

#include <watchdog.h>
#include <groundcontrol.h>
#include <groundcontrol/proto.h>

watchdog_t watchdog_data;
pthread_mutex_t watchdog_mutex = PTHREAD_MUTEX_INITIALIZER;

void *watchdog_loop(void *ptr) {
	uint8_t ret;
	int count = 0, dim;
	long delta = 0, loop_d = 0;
	struct timeval tv1, tv2;

//	sensori_t w_sensori;
	uint8_t buf[NET_BUF_SIZE];

	init_watchdog();

	while(1) {
		gettimeofday(&tv1, NULL);
		ret = get_status_watchdog(NULL);

		/////////////////
		//FAILSAFE
		if (get_configurato_watchdog() && (!get_connesso_watchdog())) {
			set_failsafe_watchdog(true);
		}
/*
		if (ret >= 64) {
			printf("failsafe ");
			ret-=64;
		}

		if (ret >= 32) {
			printf("attuatori_ok ");
			ret-=32;
		}

		if (ret >= 16) {
			printf("sensori_ok ");
			ret-=16;
		}

		if (ret >= 8) {
			printf("groundcontrol_ok ");
			ret-=8;
		}

		if (ret >= 4) {
			printf("pilota_ok ");
			ret-=4;
		}

		if (ret >= 2) {
			printf("configurato ");
			ret-=2;
		}

		if (ret == 1) {
			printf("connesso ");
		}

		printf("\n");
*/
		////////////////
/*
		read_sensori(&w_sensori);
		dim = create_message(buf,(uint8_t *)&w_sensori,sizeof(sensori_t),SEND_DATA);

			send_tcpserver(buf, dim);
			send_udpserver(buf, dim);

		if (count >= 5000000) {
			set_status_watchdog(ret & 0x0B);
//			w_sensori.battery = misura_batteria(w_sensori.battery);
//			write_batteria_sensori(&w_sensori);
			count = 0;
		}
*/

		if (count >= 1000000) {
			//set_status_watchdog(ret & 0x0B);
			set_status_watchdog(0);
			count = 0;
		}

		count += PERIODO_WATCHDOG;
		gettimeofday(&tv2, NULL);

		delta= (tv2.tv_sec - tv1.tv_sec)*(10^6) + (tv2.tv_usec - tv1.tv_usec);

		loop_d = PERIODO_WATCHDOG - delta;

		if (loop_d > 0) {
			usleep(loop_d);
		}
	}
}

void init_watchdog() {
	memset(&watchdog_data, 0, sizeof(watchdog_t));
//	watchdog_data.failsafe = false;
//	watchdog_data.failsafe_mode = RTL;
}

void set_status_watchdog(int a) {
	pthread_mutex_lock(&watchdog_mutex);
	if (a >= 64) {
		watchdog_data.failsafe = true;
		a-=64;
	} else {
		watchdog_data.failsafe = false;
	}

	if (a >= 32) {
		watchdog_data.attuatori = true;
		a-=32;
	} else {
		watchdog_data.attuatori = false;
	}

	if (a >= 16) {
		watchdog_data.sensori = true;
		a-=16;
	} else {
		watchdog_data.sensori = false;
	}

	if (a >= 8) {
		watchdog_data.groundcontrol = true;
		a-=8;
	} else {
		watchdog_data.groundcontrol = false;
	}

	if (a >= 4) {
		watchdog_data.pilota = true;
		a-=4;
	} else {
		watchdog_data.pilota = false;
	}

	if (a >= 2) {
		watchdog_data.configurato = true;
		a-=2;
	} else {
		watchdog_data.configurato = false;
	}

	if (a == 1) {
		watchdog_data.connesso = true;
	} else {
		watchdog_data.connesso = false;
	}
	pthread_mutex_unlock(&watchdog_mutex);
}

int get_status_watchdog(watchdog_t *b) {
	int a=0;

	pthread_mutex_lock(&watchdog_mutex);
	if (b != NULL) {
		b->connesso = watchdog_data.connesso;
		b->configurato = watchdog_data.configurato;
		b->pilota = watchdog_data.pilota;
		b->groundcontrol = watchdog_data.groundcontrol;
		b->sensori = watchdog_data.sensori;
		b->attuatori = watchdog_data.attuatori;
		b->failsafe = watchdog_data.failsafe;
		b->failsafe_mode = watchdog_data.failsafe_mode;
	}
	a += watchdog_data.connesso;
	a += watchdog_data.configurato<<1;
	a += watchdog_data.pilota<<2;
	a += watchdog_data.groundcontrol<<3;
	a += watchdog_data.sensori<<4;
	a += watchdog_data.attuatori<<5;
	a += watchdog_data.failsafe<<6;

	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}


void set_connesso_watchdog(bool a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.connesso = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
bool get_connesso_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.connesso;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}

void set_configurato_watchdog(bool a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.configurato = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
bool get_configurato_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.configurato;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}

void set_pilota_watchdog(bool a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.pilota = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
bool get_pilota_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.pilota;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}

void set_groundcontrol_watchdog(bool a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.groundcontrol = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
bool get_groundcontrol_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.groundcontrol;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}

void set_sensori_watchdog(bool a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.sensori = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
bool get_sensori_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.sensori;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}

void set_attuatori_watchdog(bool a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.attuatori = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
bool get_attuatori_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.attuatori;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}

void set_failsafe_watchdog(bool a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.failsafe = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
bool get_failsafe_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.failsafe;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}

void set_failsafe_mode_watchdog(uint8_t a) {
	pthread_mutex_lock(&watchdog_mutex);
	watchdog_data.failsafe_mode = a;
	pthread_mutex_unlock(&watchdog_mutex);
}
uint8_t get_failsafe_mode_watchdog() {
	bool a;
	pthread_mutex_lock(&watchdog_mutex);
	a = watchdog_data.failsafe_mode;
	pthread_mutex_unlock(&watchdog_mutex);
	return(a);
}


