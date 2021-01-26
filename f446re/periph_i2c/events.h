/*
 * events.h - definitions for the i2c event logger
 *
 * The events logger keeps a running stream of things that
 * were logged by the driver. This can illuminate control flow
 * through the peripheral driver and also be used to debug i2c calls
 * from master devices.
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

#pragma once
#include <stdint.h>

/*
 * This is some code where we're playing around with defining an
 * I2C state machine that can be used to drive interface behaviors.
 */
typedef enum {
	ADDR1_WRITE = 1,	/* Address 1 matched with R/W* = 0 */
	ADDR1_READ,			/* Address 1 matched with R/W* = 1 */
	ADDR2_WRITE,		/* Address 2 matched with R/W* = 0 */
	ADDR2_READ,			/* Address 2 matched with R/W* = 1 */
	RECV_BYTE,			/* Interface received a byte */
	SENT_BYTE,			/* Interface needs a byte to write */
	NAK,				/* NAK recieved on data transfer (write) */
	STOP,				/* STOP received on data transfer (read) */
	CALL,				/* General CALL */
	ERROR,				/* Some Error occurred */
} i2c_event_t;

struct sm_event_t {
	i2c_event_t	ev;
	uint8_t		c;
	uint8_t		d;
};

/*
 * Specific errors that are detected.
 */
#define SM_ERR_TIMEOUT			1
#define SM_ERR_OVERUNDERFLOW	2
#define SM_ERR_ARLO				4
#define SM_ERR_PEC				8
#define SM_ERR_BUSERROR			16

/* Prototypes */
/* Log an i2c event */
void sm_log_state(i2c_event_t ev, uint8_t, uint8_t);
/* Dump all i2c events that have happened since last dump. */
void sm_dump_state(void);
/* Flush (discard) all events in the buffer */
void sm_flush_state(void);
/* Return the number of events being held in the log currently */
int sm_log_size(void);

