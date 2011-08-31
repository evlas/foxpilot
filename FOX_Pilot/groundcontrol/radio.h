/*
 * radio.h
 *
 *  Created on: 26/dic/2010
 *      Author: vito
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <defines.h>
#include <config.h>

#define WLAN_LEVEL "/sys/class/net/wlan0/wireless/level"

typedef struct __comm_t {
	uint8_t rssi; //rssi 0-255
	uint32_t rx_drop_count;     /// Receive drops
	uint32_t rx_success_count;
} comm_t;

typedef struct __manual_ctrl_t {
	float roll; ///< roll -1.0 <-> 1.0
	float pitch; ///< pitch -1.0 <-> 1.0
	float yaw; ///< yaw -1.0 <-> 1.0
	float thrust; ///< thrust 0.0 <-> 1.0 ??
	uint8_t roll_manual; ///< roll control enabled auto:0, manual:1
	uint8_t pitch_manual; ///< pitch auto:0, manual:1
	uint8_t yaw_manual; ///< yaw auto:0, manual:1
	uint8_t thrust_manual; ///< thrust auto:0, manual:1
} manual_ctrl_t;

void esc_calibration(void);
void rc_calibration(void);

//re-map rc in 0-10000 value 0->0% 10000->100%
int16_t to_10000(float value);
uint16_t to_rc(int16_t value);

//rssi 0-255
uint8_t get_rssi(void);

uint32_t get_drop_rate(void);
void set_drop_rate(	uint32_t drop_count, uint32_t success_count);

#endif /* RADIO_H_ */
