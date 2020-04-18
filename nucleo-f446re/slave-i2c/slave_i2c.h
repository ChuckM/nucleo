/*
 * Slave i2c handler defines
 *
 * Defines for i2c device handlers. A device handler implements a specific
 * set of semantics for the slave device (i2c has no default semantics of
 * its own). Two examples are provided, one called "echo" and one called "emd."
 *
 * Echo simple echos back bytes that were written to the device. 
 *
 * EMD is an emulated character display. It has 8 "registers" that do various
 * things and a display state. It represents how you would implement a more
 * complex set of device semantics.
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

typedef struct {
	uint8_t		(*send)(void *);
	void		(*recv)(void *, uint8_t db);
	void		(*start)(void *, uint8_t rw);
	void		(*stop)(void *, uint8_t err);
	uint16_t	addr;
	void		*state;
} i2c_handler_t;

#define HANDLE_MODE_SENDING		1
#define HANDLE_MODE_RECEIVING	2

#define SI2C_RETURN_SUCCESS			 	 0
#define SI2C_RETURN_NO_MEMORY			-1
#define SI2C_RETURN_MISSING_H1_STATE	-2
#define SI2C_RETURN_MISSING_H2_STATE	-3

/* setup the slave I2C interface, returns 0 on success < 0 on failure */
int setup_i2c(uint8_t addr1, const i2c_handler_t *h1, void *h1_state,
			   uint8_t addr2, const i2c_handler_t *h2, void *h2_state);

