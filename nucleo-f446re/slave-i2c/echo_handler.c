/*
 * Echo handler - An i2c handler driver for slave devices
 *
 * This simple handler stores bytes that are written to the device
 * and returns them when the device is read.
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
#include "echo_handler.h"

static void echo_start(void *state, uint8_t rw);
static uint8_t echo_send(void *);
static void echo_recv(void *state, uint8_t db);
static void echo_finish(void *state, uint8_t err);

/*
 * Prototype handler structure for the echo
 * device.
 */
const i2c_handler_t echo_handler = {
	.send = echo_send,
	.recv = echo_recv,
	.start = echo_start,
	.stop = echo_finish,
	.addr = 0,
	.state = NULL
};


/*
 * Allocate and Initialize the custom "state" structure
 *
 * This allocates memory for it and then fills in all the
 * pointer values.
 *
 * Note: it returns NULL if any of the allocations fail.
 */
struct echo_state_t *
echo_init(uint16_t buf_size)
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
	state->rb->buffer = calloc(buf_size, 1);
	if (state->rb->buffer == NULL) {
		free(state->rb);
		free(state);
		return NULL;
	}
	state->rb->size = buf_size;
	ringbuffer_flush(state->rb);
	return state;
}

/*
 * Send a byte from the echo chamber to the master.
 *
 * The ringbuffer is FIFO so this will nominally just return
 * what you previously sent it, caveat if you are asked to
 * send more than you have received it sends 0xff instead and
 * notes it as a send error.
 */
static uint8_t
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

/*
 * Receive a byte from the master and put it into the echo chamber.
 *
 * As noted above the ringbuffer is a FIFO. It is also limited in
 * size set at allocation, so if you send more data than will fit before
 * reading back from the echo chamber it starts dropping bytes on
 * the floor (they are discarded, and receive errors are recorded
 * in the state data.
 */
static void
echo_recv(void *s, uint8_t db)
{
	struct echo_state_t *state = (struct echo_state_t *)(s);
	if (ringbuffer_available(state->rb) == 0) {
		state->recv_errs++;
		return;
	} else {
		state->bytes_recv++;
	}
	ringbuffer_put(state->rb, db);
}

/*
 * Start a transaction, pretty boring it just increments the
 * transaction count.
 */
static void
echo_start(void *s, uint8_t rw)
{
	struct echo_state_t *state = (struct echo_state_t *)(s);

	if (rw == HANDLE_MODE_RECEIVING) {
		state->receive_count++;
	} else {
		state->send_count++;
	}
}

/*
 * Finish a transaction 
 *
 * If there was an error make a note of it.
 */
static void
echo_finish(void *s, uint8_t err)
{
	struct echo_state_t *state = (struct echo_state_t *)(s);

	if (err) {
		state->err_count++;
		fprintf(stderr, "Error transaction: %d\n", err);
	} 
	state->total++;
}
