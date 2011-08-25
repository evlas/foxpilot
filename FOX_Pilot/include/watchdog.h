/*
 * watchdog.h
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>

typedef struct __watchdog_t {
  bool     connesso;            //1
  bool     configurato;         //2
  bool     pilota;              //4
  bool     groundcontrol;       //8
  bool     sensori;             //16
  bool     attuatori;           //32
  bool     failsafe;            //64
  uint8_t  failsafe_mode;
} watchdog_t;

pthread_mutex_t watchdog_mutex;

extern watchdog_t watchdog_data;

void *watchdog_loop(void *ptr);

void init_watchdog();
void deinit_watchdog();

void set_status_watchdog(int);
int get_status_watchdog(watchdog_t *b);

void set_connesso_watchdog(bool);
bool get_connesso_watchdog();

void set_configurato_watchdog(bool);
bool get_configurato_watchdog();

void set_pilota_watchdog(bool);
bool get_pilota_watchdog();

void set_groundcontrol_watchdog(bool);
bool get_groundcontrol_watchdog();

void set_sensori_watchdog(bool);
bool get_sensori_watchdog();

void set_attuatori_watchdog(bool);
bool get_attuatori_watchdog();

void set_failsafe_watchdog(bool);
bool get_failsafe_watchdog();

void set_failsafe_mode_watchdog(uint8_t);
uint8_t get_failsafe_mode_watchdog();

#endif /* WATCHDOG_H_ */
