/*
 * ring.h
 *
 *  Created on: 26/dic/2010
 *      Author: vito
 */

#ifndef RING_H_
#define RING_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

#define RING_LEN 100

typedef struct __ring_buffer_t {
	int   pos_read;
	int   pos_write;
	int   count;
	uint8_t  buffer[RING_LEN];
} ring_buffer_t;

static inline void init_ring_buffer(ring_buffer_t *ring) {
	ring->pos_read = 0;
	ring->pos_write = 0;
	ring->count = 0;
	memset(ring->buffer, 0, RING_LEN);
}

static inline int read_ring_buffer(ring_buffer_t *ring, uint8_t *buf, int count){
	int i;

	if (ring->count < count) {
		return(0);
	}

	for(i=0;i<count;i++) {
		if(ring->pos_read == RING_LEN) {
			ring->pos_read = 0;
		}
		buf[i]=ring->buffer[ring->pos_read];
		ring->pos_read++;
	}

	ring->count -= count;

	return(count);
}

static inline int write_ring_buffer(ring_buffer_t *ring, uint8_t *buf, int count) {
	int i;

	if ((RING_LEN-ring->count) < count) {
		return(0);
	}

	for(i=0;i<count;i++) {
		if(ring->pos_write == RING_LEN) {
			ring->pos_write = 0;
		}
		ring->buffer[ring->pos_write]=buf[i];
		ring->pos_write++;
	}

	ring->count += count;

	return(count);
}


#endif /* RING_H_ */
