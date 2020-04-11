/*
 * Simple Ringbuffer type for interrupt functions
 *
 * There are lots of implementations of ring buffers, its a pretty common
 * data structure. I kept writing and re-writing this code. I'm lazy so I
 * put it into an API and then just reuse this code.
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
 * Ringbuffer type. Limited by uint16_t indices to buffers
 * of 64K bytes or less.
 */
typedef struct {
	uint8_t		*buffer;	/* buffer for the data */
	uint16_t	size;
	uint16_t	next;		/* Insert here. */
	uint16_t	cur;		/* Pull from here. */
	uint16_t	err;		/* Under/Overflow errors */
} ringbuffer_t;

/* Error behavior: On empty, return error byte */
#define RINGBUFFER_EMPTY_BYTE	0xee

/* Read (extract) a byte from the ringbuffer */
uint8_t ringbuffer_get(ringbuffer_t *buf);
/* Write (insert) a byte into the ringbuffer */
void ringbuffer_put(ringbuffer_t *buf, uint8_t data);
/* Return the number of bytes in the ringbuffer */
uint16_t ringbuffer_bytes(ringbuffer_t *buf);
