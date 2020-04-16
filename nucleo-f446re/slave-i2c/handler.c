/*
 * handler.c - An i2c handler system for slave devices
 *
 * Much like USB devices, when you're the slave you don't get to drive.
 * Writing code to support that can be a bit tricky. I've chosen to
 * create the notion of a 'handler' which can be plugged into the
 * i2c slave driver and together they give you some feature. The
 * goal is maximum code re-use.
 *
 * BSD 2-Clause License
 * 
 * Copyright (C) 2013-2020 Chuck McManis <cmcmanis@mcmanis.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "ring.h"
#include "slave_i2c.h"

//typedef struct {
//	void	*	(*init)(void);
//	uint8_t		(*send)(void *);
//	void		(*recv)(void *, uint8_t db);
//	void		(*start)(void *, uint8_t rw);
//	void		(*stop)(void *, uint8_t err);
//	uint16_t	addr;
//	uint8_t		mode; 	/* IDLE, READING, WRITING */
//	void		*state;
//} i2c_handler_t;

void *echo_init(void);
uint8_t echo_send(void *);
void echo_recv(void *state, uint8_t db);
void echo_start(void *state, uint8_t rw);
void echo_stop(void *state, uint8_t err);

i2c_handler_t echo_handler = {
	.init = echo_init,
	.send = echo_send,
	.recv = echo_recv,
	.start = echo_start,
	.stop = echo_stop,
	.addr = 0,
	.mode = HANDLE_MODE_IDLE,
	.state = NULL
};

/*
 * Echo handler
 *
 * This simple handler stores bytes that are written to the device
 * and returns them when the device is read.
 *
 * Note, this is dynamically allocated because that makes it "easy"
 * to have two echo responders, but mostly it also just makes the
 * code more flexible.
 */

#define ECHO_BUFSIZE	256

struct echo_state_t {
	ringbuffer_t	*rb;
	uint32_t		bytes_sent, bytes_recv;
	uint32_t		send_errs, recv_errs;
	uint8_t			mode;
};

void *
echo_init(void)
{
	struct echo_state_t *state;

	state = calloc(1, sizeof(struct echo_state_t));
	if (state == NULL) {
		return NULL;
	}
	state->rb = calloc(1, sizeof(ringbuffer_t));
	if (state->rb == NULL) {
		free(state);
		return NULL;
	}
	state->rb->buffer = calloc(ECHO_BUFSIZE, 1);
	if (state->rb->buffer == NULL) {
		free(state->rb);
		free(state);
		return NULL;
	}
	state->rb->size = ECHO_BUFSIZE;
	ringbuffer_flush(state->rb);
	return (void *) state;
}

uint8_t
echo_send(void *s)
{
	struct echo_state_t *state = (struct echo_state_t *)(s);
	uint8_t db;

	if (ringbuffer_holding(state->rb)) {
		db = ringbuffer_get(state->rb);
		state->bytes_sent++;
	} else {
		db = 0xff;
		state->send_errs++;
	}
	return db;
}

void
echo_recv(void *s, uint8_t db)
{
	struct echo_state_t *state = (struct echo_state_t *)(s);
	if (ringbuffer_available(state->rb) == 0) {
		state->recv_errs++;
		return;
	}
	ringbuffer_put(state->rb, db);
}

void
echo_start(void *s, uint8_t rw)
{
	struct echo_state_t *state = (struct echo_state_t *)(s);

	state->mode = rw;
}

void
echo_stop(void *s, uint8_t err)
{
	struct echo_state_t *state = (struct echo_state_t *)(s);

	state->mode = 0;
	if (err) {
		fprintf(stderr, "Error transaction: %d\n", err);
	}
}


