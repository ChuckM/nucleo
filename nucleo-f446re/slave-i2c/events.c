/*
 * Simple event logging tool for debugging
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
#include <stdint.h>
#include <stdlib.h>
#include "events.h"

#define RED_TEXT 		"\033[31;40;1m"
#define GREEN_TEXT		"\033[32;40;1m"
#define YELLOW_TEXT		"\033[33;40;1m"
#define DEFAULT_TEXT	"\033[0m"

/*
 * Return a text message to describe the event
 */
char *
log_message(event_t e)
{
	switch (e) {
		default:
			return "Unknown";
		case NO_EVENT:
			return "No Event";
		case ADDRESS_RECEIVED:
			return "Address Received";
		case RECV_FULL:
			return "Received Byte";
		case XMIT_EMPTY:
			return (RED_TEXT "Sending Fill Byte (buffer empty)" DEFAULT_TEXT);
		case BYTE_READ:
			return (YELLOW_TEXT "Byte read" DEFAULT_TEXT);
		case BYTE_SENT:
			return (YELLOW_TEXT "Byte sent" DEFAULT_TEXT);
		case BYTE_FINISHED:
			return "Byte Finished";
		case SLAVE_XMIT:
			return "Slave Transmit";
		case SLAVE_RECEIVE:
			return "Slave Receive";
		case BUS_ERROR:
			return "Bus Error";
		case TIMEOUT:
			return "Timeout";
		case ACK_FAIL:
			return "No Acknowledge (NAK)";
		case OVERRUN:
			return "Data Over or Underrun";
		case SLAVE_STOP:
			return "Slave STOP";
		case ADDR1_MATCH:
			return "Address 1 Matched";
		case ADDR2_MATCH:
			return "Address 2 Matched";
		case GENERAL_CALL:
			return "General Call";
		case BUS_ERROR_RESET:
			return "Reset Bus Error";
		case BUS_ERROR_DETECTED_AND_RESET:
			return "Bus Error Detected and Reset";
		case ACK_FAIL_RESET:
			return "ACK Failure Reset";
		case ACK_FAIL_DETECTED_AND_RESET:
			return "ACK Failure Detected and Reset";
		case DUMMY_BYTE:
			return "Dummy Byte";
		case SLAVE_TRANSMIT_REQUEST:
			return "Slave Requested to Transmit";
		case SLAVE_RECEIVE_REQUEST:
			return "Slave Requested to Receive";
		case XMIT_ACK_IS_SET:
			return "TxNE: ACK flag is set";
		case XMIT_ACK_IS_NOT_SET:
			return "TxNE: ACK is NOT set.";
		case XMIT_AF_IS_SET:
			return "TxNE: AF is set.";
		case XMIT_AF_IS_NOT_SET:
			return "TxNE: AF is NOT set.";
		case XMIT_BTF_IS_SET:
			return "TxNE: Byte Finished is set.";
		case XMIT_BTF_IS_NOT_SET:
			return "TxNE: Byte Finished is NOT set.";
		case BTF_ERROR:
			return (RED_TEXT "BTF set but not Rx or Tx!" DEFAULT_TEXT);
		case STOP_AND_READ_IN_EVENT:
			return "Stop and read from a STOPF event";
		case ER_STOP:
			return "Stop as an ER event";
		case AF_ALSO_SET:
			return "AF also set during STOP";
		case BUS_RESTART:
			return "Repeated START";
	}
}

#define EVENT_BUFSIZE	1024
event_t event_queue[EVENT_BUFSIZE];
static uint16_t	event_nxt;
static uint16_t	event_cur;

extern void uart_puts(char *);

void
log_event(event_t e)
{
	if (((event_nxt + 1) % EVENT_BUFSIZE) == event_cur) {
		uart_puts("Event Queue full, Event Discarded\n");
	}
	event_queue[event_nxt] = e;
	event_nxt = (event_nxt + 1) % EVENT_BUFSIZE;
}

int
logged_events(void)
{
	return ((event_nxt < event_cur) ? ((event_nxt + EVENT_BUFSIZE) - event_cur)
								    : (event_nxt - event_cur));
}

int
dump_event(void)
{
	if (event_nxt == event_cur) {
		return 0;
	}
	printf("EVT: %s\n", log_message(event_queue[event_cur]));
	event_cur = (event_cur + 1) % EVENT_BUFSIZE;
	return 1;
}
