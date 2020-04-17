/*
 * I2C Event logging
 *
 * This code is called (if #defined) by the i2c driver. It can help
 * you figure out the flow of execution through the various interrupts.
 *
 * Events have three parameters:
 * 		- The type (like SENT_BYTE)
 * 		- A call instance (decimal number) I use this to distinguish
 * 		  different places where the same I2C function (like STOP) may
 * 		  be called from.
 * 		- And a data byte, data associated with the I2C function like
 * 		  the byte received or sent, or the address received. In the
 * 		  case of errors, this byte is the error number which is defined
 * 		  below.
 *
 * Events are held in a ring buffer, if you capture more events than you
 * have space in the ring buffer, the earlier ones are discarded. In that
 * way when you dump the buffer you get the last 'n' events where n is a
 * maximum of the buffer size - 1. 
 *
 * Planned future work is to have one of the 32 bit timers free running at
 * 10MHz and copying its value into a uint32_t when the event fires, giving
 * something of an idea about the time between events. Its a poor substitute
 * for a good tracing probe but it can work in a pinch.
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
#include "../common.h"
#include "events.h"

/***************************************
 *  E X P E R I M E N T A L   C O D E  *
 ***************************************/

/*
 * This is the event buffer.
 */

#define SM_EVENT_BUFSIZE	300

struct sm_event_t sm_events[SM_EVENT_BUFSIZE];

volatile int	sm_cur_event;
volatile int 	sm_nxt_event;


/*
 * Capture the state change of the interface.
 *
 * The parameter 'c' is which call location originated the state
 * change. This is to  track down what part of the code is executed
 * and what part isn't.
 *
 * The parameter 'd' is any data associated with the event so the
 * address, or the error, or the actual data.
 */
void
sm_log_state(i2c_event_t ev, uint8_t c, uint8_t d)
{
	sm_events[sm_nxt_event].ev = ev;
	sm_events[sm_nxt_event].c = c;
	sm_events[sm_nxt_event].d = d;
	sm_nxt_event = (sm_nxt_event + 1) % SM_EVENT_BUFSIZE;
}

/*
 * Flush the buffer
 *
 * This just sets current = next which is an empty buffer as
 * far as the ringbuffer semantics go.
 */
void
sm_flush_state(void)
{
	sm_cur_event = sm_nxt_event;
}

/*
 * Return the number of events being held in the buffer.
 */
int
sm_log_size()
{
	return ((sm_nxt_event < sm_cur_event) ? 
			(sm_nxt_event + SM_EVENT_BUFSIZE) - sm_cur_event :
			sm_nxt_event - sm_cur_event);
}

/*
 * Dump out a sequence of state changes captured since the last time
 * states were dumped out. The sister call sm_flush_state(void) just
 * sets the pointers to equal thus presenting an empty buffer.
 *
 * Note: I don't call printf as this code may be called at interrupt time
 * which printf really doesn't like.
 */
void
sm_dump_state(void)
{
	char buf[60];
	uart_puts("[ ");
	while (sm_cur_event != sm_nxt_event) {
		struct sm_event_t *ce;

		ce = &sm_events[sm_cur_event];
		switch (sm_events[sm_cur_event].ev) {
			default:
				sprintf(buf, "UNK(%d)", (int) ce->ev);
				uart_puts(buf);
				break;
			case ADDR1_WRITE:
				sprintf(buf,"ADDR1_WRITE<0x%02X>", ce->d);
				uart_puts(buf);
				break;
			case ADDR1_READ:
				sprintf(buf,"ADDR1_READ<0x%02X>", ce->d);
				uart_puts(buf);
				break;
			case ADDR2_WRITE:
				sprintf(buf,"ADDR2_WRITE<0x%02X>", ce->d);
				uart_puts(buf);
				break;
			case ADDR2_READ:
				sprintf(buf,"ADDR2_READ<0x%02X>", ce->d);
				uart_puts(buf);
				break;
			case RECV_BYTE:
				sprintf(buf,"RECV_BYTE%d<0x%02X>", ce->c, ce->d);
				uart_puts(buf);
				break;
			case SENT_BYTE:
				sprintf(buf,"SENT_BYTE%d<0x%02X>", ce->c, ce->d);
				uart_puts(buf);
				break;
			case NAK:
				sprintf(buf,"NAK%d", ce->c);
				uart_puts(buf);
				break;
			case CALL:
				uart_puts("CALL");
				break;
			case STOP:
				uart_puts("STOP");
				break;
			case ERROR:
				switch(ce->d) {
					case SM_ERR_TIMEOUT:
						sprintf(buf,"ERROR%d<TIMEOUT>", ce->c);
						uart_puts(buf);
						break;
					case SM_ERR_ARLO:
						sprintf(buf,"ERROR%d<ARLO>", ce->c);
						uart_puts(buf);
						break;
					case SM_ERR_OVERUNDERFLOW:
						sprintf(buf,"ERROR%d<OVERUNDERFLOW>", ce->c);
						uart_puts(buf);
						break;
					case SM_ERR_PEC:
						sprintf(buf,"ERROR%d<PEC>", ce->c);
						uart_puts(buf);
						break;
					case SM_ERR_BUSERROR:
						sprintf(buf,"ERROR%d<BUSERROR>", ce->c);
						uart_puts(buf);
						break;
					default:
						sprintf(buf,"ERROR%d<0x%02X>", ce->c, ce->d);
						uart_puts(buf);
						break;
				}
				break;
		}
		sm_cur_event = (sm_cur_event + 1) % SM_EVENT_BUFSIZE;
		if (sm_cur_event != sm_nxt_event) {
			uart_puts(" => ");
		} else {
			uart_puts(" ]");
		}
	}
}
