/*
 * manual.c
 *
 *  Created on: 02/apr/2011
 *      Author: vito
 */

#include <pilota.h>
#include "manual.h"
#include <groundcontrol.h>
#include <groundcontrol/radio.h>

int processFlightControlmanual() {}

/*
int processFlightControlmanual() {
	int i;

	if (pilota_data.groundcontrol.engage) {
		for (i=0;i < RADIO_NCHAN;i++) {
#if POLOLU
			if((i<(POLOLU*POLOLU_NCHAN)) &&  (pilota_data.attuatori.pololu_chan[i].value != pilota_data.groundcontrol.radio[i].value)){
				pilota_data.attuatori.pololu_chan[i].value = pilota_data.groundcontrol.radio[i].value;
				pilota_data.attuatori.pololu_change += 1<<i;
			}
#endif
#if BRUSHLESS
			if ((i>=(POLOLU*POLOLU_NCHAN)) && (i<(BRUSHLESS*BRUSHLESS_NCHAN)) &&  (pilota_data.attuatori.brushless_chan[i].value != pilota_data.groundcontrol.radio[i].value)) {
				pilota_data.attuatori.brushless_chan[i-(POLOLU*POLOLU_NCHAN)].value = pilota_data.groundcontrol.radio[i].value;
			}
			pilota_data.attuatori.brushless_change += 1<<(i-POLOLU*POLOLU_NCHAN);
#endif
		}
	}

	return(0);
}
*/


