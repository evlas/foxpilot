// http://linux.die.net/man/2/uselib
// http://www.kernel.org/doc/man-pages/online/pages/man3/dlopen.3.html
// http://souptonuts.sourceforge.net/code/dlopen.c.html

/*
 * main.c
 *
 *  Created on: 28/lug/2011
 *      Author: Vito Ammirata
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>

#include <defines.h>
#include <config.h>

#include <watchdog.h>
#include <groundcontrol.h>
#include <sensori.h>
#include <attuatori.h>
#include <pilota.h>

#include <groundcontrol/proto.h>
#include <groundcontrol/TCP_Server/tcpserver.h>
#include <groundcontrol/UDP_Server/udpserver.h>

#include <sensori/arduimu/arduimu.h>
#include <sensori/batteria/mybatteria.h>

#include <attuatori/pololu/pololu.h>

int main(int argc, char *argv[]) {
	pthread_t pthr_watchdog, pthr_groundcontrol[3], pthr_sensori[2], pthr_attuatori[1], pthr_pilota;

	//Load della configurazione
//	load_configuration();
//	save_configuration();

	//Avvio il watchdog
	if (pthread_create(&pthr_watchdog, NULL, watchdog_loop, NULL)){
		printf("ERROR; pthread_create(watchdog)\n");
		exit(errno);
	}
	//Avvio il controllo da terra TCP e UDP
	init_groundcontrol();
	if (pthread_create(&pthr_groundcontrol[0], NULL, protocol_loop, NULL)){
		printf("ERROR; pthread_create(groundcontrol(protocol))\n");
		exit(errno);
	}
//	if (pthread_create(&pthr_groundcontrol[1], NULL, tcpserver_loop, NULL)){
//		printf("ERROR; pthread_create(groundcontrol(tcpserver))\n");
//		exit(errno);
//	}
	if (pthread_create(&pthr_groundcontrol[2], NULL, udpserver_loop, NULL)){
		printf("ERROR; pthread_create(groundcontrol(udpserver))\n");
		exit(errno);
	}

	while (get_sys_state_status() < MAV_STATE_STANDBY) {
		if (get_sys_state_status() == 0) {
			set_sys_state_status(MAV_STATE_BOOT);
		}

		send_mav_statustext(0, "Check Configurazione", 20);

		sleep(1);

		//printf("get_sys_state_status() %d\n",get_sys_state_status());
	}

	//Avvio la gestione degli attuatori
	init_attuatori();
	conf_pololu(POLOLU_DEVICE, POLOLU_RATE, POLOLU_DATABITS, POLOLU_STOPBITS, POLOLU_PARITY);
	if (pthread_create(&pthr_attuatori[0], NULL, pololu_loop, NULL)){
		printf("ERROR; pthread_create(attuatori(pololu))\n");
		exit(errno);
	}
	if((int)get_param_value(PARAM_ESC_CALIBRATION) == 1) {
		esc_calibration();
		set_param_value(PARAM_ESC_CALIBRATION, 0.0);
	}

	//Avvio la lettura dai sensori
	init_sensori();
	conf_arduimu(ARDUIMU_DEVICE, ARDUIMU_RATE, ARDUIMU_DATABITS, ARDUIMU_STOPBITS, ARDUIMU_PARITY);
	if (pthread_create(&pthr_sensori[0], NULL, arduimu_loop, NULL)){
		printf("ERROR; pthread_create(sensori(arduimu))\n");
		exit(errno);
	}
//	conf_mybatteria(BATTERYCHANV, BATTERYCHANA, R1_battery, R2_battery, diodo_battery, Aref_battery, Periodo_battery);
//	if (pthread_create(&pthr_sensori[1], NULL, mybatteria_loop, NULL)){
//		printf("ERROR; pthread_create(sensori(batteria))\n");
//		exit(errno);
//	}

	//Avvio logica di volo
	if (pthread_create(&pthr_pilota, NULL, pilota_loop, NULL)){
		printf("ERROR; pthread_create(pilota)\n");
		exit(errno);
	}

	pthread_join(pthr_watchdog, NULL);
	pthread_join(pthr_groundcontrol[0], NULL);
//	pthread_join(pthr_groundcontrol[1], NULL);
	pthread_join(pthr_groundcontrol[2], NULL);
	pthread_join(pthr_sensori[0], NULL);
//	pthread_join(pthr_sensori[1], NULL);
	pthread_join(pthr_attuatori[0], NULL);
	pthread_join(pthr_pilota, NULL);

	return(0);
}

