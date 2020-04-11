/*
 * events.h - definitions for the i2c event logger
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


typedef enum {
	NO_EVENT,
	ADDRESS_RECEIVED,
	RECV_FULL,
	XMIT_EMPTY,
	BYTE_READ,
	BYTE_SENT,
	BYTE_FINISHED,
	SLAVE_XMIT,
	SLAVE_RECEIVE,
	TIMEOUT,
	OVERRUN,
	SLAVE_STOP,
	ADDR1_MATCH,
	ADDR2_MATCH,
	GENERAL_CALL,
	BUS_ERROR,
	BUS_RESTART,
	BUS_ERROR_RESET,
	BUS_ERROR_DETECTED_AND_RESET,
	ACK_FAIL,
	ACK_FAIL_RESET,
	ACK_FAIL_DETECTED_AND_RESET,
	DUMMY_BYTE,
	SLAVE_TRANSMIT_REQUEST,
	SLAVE_RECEIVE_REQUEST,
	XMIT_ACK_IS_SET,
	XMIT_ACK_IS_NOT_SET,
	XMIT_AF_IS_SET,
	XMIT_AF_IS_NOT_SET,
	XMIT_BTF_IS_SET,
	XMIT_BTF_IS_NOT_SET,
	STOP_AND_READ_IN_EVENT,
	BTF_ERROR,
	ER_STOP,
	AF_ALSO_SET,
} event_t;

/* Put a new event in the log */
void log_event(event_t e);
/* return a text string for the event */
char *log_message(event_t e);
/* How many events have been logged but not seen? */
int logged_events(void);
/* Dump the next event, returns 0 if there are no events to dump */
int dump_event(void);
