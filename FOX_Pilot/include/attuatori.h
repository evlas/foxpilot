/*
 * attuatori.h
 *
 *  Created on: 27/05/2011
 *      Author: Vito Ammirata
 */

#ifndef ATTUATORI_H_
#define ATTUATORI_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include <defines.h>
#include <config.h>

#ifndef NUMS_ATTUATORI
#define NUMS_ATTUATORI 8
#endif

typedef struct __attuatori_t {
	int      id[NUMS_ATTUATORI];
	int16_t  min[NUMS_ATTUATORI];
	int16_t  zero[NUMS_ATTUATORI];
	int16_t  max[NUMS_ATTUATORI];
	int16_t  value[NUMS_ATTUATORI];
} attuatori_t;

attuatori_t     attuatori_data;
pthread_mutex_t attuatori_mutex;
pthread_cond_t  attuatori_cond;

//MANGLER
void init_attuatori();
void deinit_attuatori();

void write_attuatori(attuatori_t *src);
attuatori_t *read_attuatori(attuatori_t *dst);

void set_attuatori(attuatori_t *src);
attuatori_t *get_attuatori(attuatori_t *dst);


//void write_attuatore(attuatori_t *src, int attuatore);
//attuatori_t *read_attuatore(attuatori_t *dst, int attuatore);

//attuatori_t *get_attuatore(attuatori_t *dst, int attuatore);

#endif /* ATTUATORI_H_ */
