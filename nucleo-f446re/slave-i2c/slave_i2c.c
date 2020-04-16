/*
 * Example of setting up a slave I2C device
 *
 * I use this to talk to a Wemos D1 (ESP8266 board), more like it talks to
 * me and then I tell it I have something to say and it asks me to pass
 * that info along.
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
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "../common.h"
#include "ring.h"
#include "events.h"
#include "slave_i2c.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#define RED_TEXT 		"\033[31;40;1m"
#define GREEN_TEXT		"\033[32;40;1m"
#define YELLOW_TEXT		"\033[33;40;1m"
#define DEFAULT_TEXT	"\033[0m"
#define CLEAR_LINE		"\033[2K"
#define CLEAR_SCREEN	"\033[2J"

/*
 * One handler for each address
 */
static i2c_handler_t	*(dev[2]);

/***************************************
 *  E X P E R I M E N T A L   C O D E  *
 ***************************************/

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

/*
 * Specific errors that are detected.
 */
#define SM_ERR_TIMEOUT			1
#define SM_ERR_OVERUNDERFLOW	2
#define SM_ERR_ARLO				4
#define SM_ERR_PEC				8
#define SM_ERR_BUSERROR			16

#define SM_EVENT_BUFSIZE	300
struct sm_event_t {
	i2c_event_t	ev;
	uint8_t		c;
	uint8_t		d;
} sm_events[SM_EVENT_BUFSIZE];

volatile int	sm_cur_event;
volatile int 	sm_nxt_event;

void sm_log_state(i2c_event_t ev, uint8_t, uint8_t);
void sm_dump_state(void);

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
 * Dump out a sequence of state changes captured since the last time
 * states were dumped out. The sister call sm_flush_state(void) just
 * sets the pointers to equal thus presenting an empty buffer.
 */
void
sm_dump_state(void)
{
	char buf[40];
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
			uart_puts(" |");
		}
	}
}

void send_byte(int d);
void read_byte(int c);

/*
 * For test type 1, we just store data in a ring buffer
 * when written, and send it back when read. If more bytes
 * are read than have been sent we send the empty byte (0xee)
 */
static uint8_t i2c_buffer[256];
ringbuffer_t txb = {
	.buffer = &i2c_buffer[0],
	.size = 256,
	.next = 0,
	.cur = 0,
	.err = 0,
};

/*
 * For test type 2, we have a bank of 16 pseudo registers and
 * one memory area of 32 bytes (simulates something like a 16x2
 * Character display device).
 */
struct p_reg_t {
	uint16_t	val;
	uint8_t		changed;
} reg_bank[16];

int current_reg = 0;

#define MEM_REG_SIZE	32

struct mem_reg_t {
	uint8_t		buf[MEM_REG_SIZE];
	uint8_t		cursor;
	uint8_t		changed;
} mem_reg;

/*
 * Interrupt service routine for I2C interface #1 Errors
 */
void
i2c1_er_isr(void)
{
	uint16_t sr1 = I2C_SR1(I2C1);

	if ((sr1 & I2C_SR1_STOPF) != 0) {
		sm_log_state(STOP, 0, 0);
		I2C_CR1(I2C1) |= I2C_CR1_STOP;
		if ((sr1 & I2C_SR1_AF) != 0) {
			sm_log_state(NAK, 1, 0);
		}
	}
	if ((sr1 & I2C_SR1_TIMEOUT) != 0) {
		sm_log_state(ERROR, 0, SM_ERR_TIMEOUT);
	}
	if ((sr1 & I2C_SR1_OVR) != 0) {
		sm_log_state(ERROR, 0, SM_ERR_OVERUNDERFLOW);
	}
	/* This appears to be "NAK" which is literally "No Acknowledge"
	 * and this is described as "Set by hardware when no ack is returned."
	 */
	if ((sr1 & I2C_SR1_AF) != 0) {
		I2C_SR1(I2C1) = I2C_SR1(I2C1) & ~(I2C_SR1_AF);
		sm_log_state(NAK, 2, 0);

		/* stop after no acknowledge (NAK) */
		sm_log_state(STOP, 0, 0);
		I2C_CR1(I2C1) |= I2C_CR1_STOP;
	}
	/* Note this happens often if you don't have the noise filter set */
	if ((sr1 & I2C_SR1_BERR) != 0) {
		I2C_SR1(I2C1) = I2C_SR1(I2C1) & ~(I2C_SR1_BERR);
		sm_log_state(ERROR, 0, SM_ERR_BUSERROR);
	}
}

uint8_t	__addr = 0;
int get_reg = 0;

/*
 * Interrupt service routine for I2C interface #1 Events
 *
 * For a given I2C device there can be up to two active addresses
 * which allows the port to respond in two different ways distinguished
 * by address. 
 */
void
i2c1_ev_isr(void)
{
	uint16_t 		sr1, sr2;	/* status register copies */
	uint8_t			db;			/* 8 bit data byte accumulator */
	i2c_handler_t	*h = NULL;	/* Handler to use */
	
	/*
	 * Read the status registers. 
	 *
	 * Note that a copy is cached because some bits are reset when you
	 * read the status register. We don't want to miss processing for
	 * everything that happened.
	 *
	 * Also, apparently the DUALF flag only gets reset by hardware on
	 * a ADDR or STOP. The question then is if it persists across calls.
	 *
	 * XXX: Also can we safely read SR2 during the DATA phase before a STOP
	 * to "remind us" which address is in use?
	 *
	 * XXX: Audit reset behavior to identify the risk here.
	 */
	sr1 = I2C_SR1(I2C1);

	/*
 	 * Handle the START of an I2C transaction (ADDR, Data, Data, Data, STOP)
 	 * We get an ADDR event, and from TRA we know if we're going to be sending
 	 * or receiving data.
 	 * 
 	 * If sending, we send the current byte in the XMIT buffer waiting to 
 	 * be sent.
 	 *
 	 * If receiving, we wait for the one byte that has to be part of the
 	 * transaction and then we're done.
 	 *
 	 * Note: Sadly we have to spin wait on recieve because if that first
 	 * byte is the only byte we're going to get, the master will NAK it and
 	 * the ST Micro I2C implementation won't set BTF to let us know it
 	 * arrived. 
 	 */
	if ((sr1 & I2C_SR1_ADDR) != 0) {
		/* This resets teh ADDR flag per the reference manual */
		sr2 = I2C_SR2(I2C1);
		/* set which handle to use */
		h = ((sr2 & I2C_SR2_DUALF) == 0) ? dev[0] : dev[1];
		if (h == NULL) {
			/* handler not defined */
			/* XXX: do a STOP here and log error */
			return;
		}

		if ((sr2 & I2C_SR2_TRA) != 0) {
			/*
			 * If the TRA bit is non zero we're transmitting 
			 * data *to* the master
			 */
			sm_log_state(ADDR1_READ, 1, h->addr);
			/* XXX this is redundant, but necessary? */
			h->mode = HANDLE_MODE_SENDING;
			(*h->start)(h->state, HANDLE_MODE_SENDING);
			/* XXX: is there a race here? or will TxE always be true */
			if (sr1 & I2C_SR1_TxE) {
				db = (*h->send)(h->state);
				sm_log_state(SENT_BYTE, 1, db); 
				I2C1_DR = db; /* send the actual byte */
			}
		} else {
			/*
			 * When the TRA bit is zero we're receiving
			 * data *from* the master
			 */
			sm_log_state(ADDR1_WRITE, 1, h->addr);
			h->mode = HANDLE_MODE_RECEIVING;
			(*h->start)(h->state, HANDLE_MODE_RECEIVING);

			/* wait for the first byte (may be NAK'd) */
			do {
				sr1 = I2C1_SR1;
			} while ((sr1 & (I2C_SR1_RxNE | I2C_SR1_TIMEOUT)) == 0);
			if ((sr1 & I2C_SR1_RxNE) != 0) {
				db = I2C1_DR;
				(*h->recv)(h->state, db);
				sm_log_state(RECV_BYTE, 1, db);
			} else {
				/* XXX: should have handle versions of error codes */
				(*h->stop)(h->state, SM_ERR_TIMEOUT);
				sm_log_state(ERROR, 2, SM_ERR_TIMEOUT);
			}
		}
	}

	/*
	 * Handle the END of an I2C transaction, the STOP part.
	 *
	 * If we were receiving we pick up the last byte that was sent to us
	 * and reset the STOP condition.
	 *
	 * If we are transmitting we set the STOP condition? (Not sure)
	 * XXX we need to make sure we distinguish which address is active
	 */
	if ((sr1 & I2C_SR1_STOPF) != 0) {
		sr2 = I2C_SR2(I2C1);
		/* set which handle to use */
		h = ((sr2 & I2C_SR2_DUALF) == 0) ? dev[0] : dev[1];
		if (h == NULL) {
			/* handler not defined */
			/* XXX: do a STOP here and log error */
			return;
		}
		/* XXX this is where it is critical, if a byte is waiting who
		 * was it intended for, address 1 or address 2?
		 */
		if ((sr1 & I2C_SR1_RxNE) != 0) {
			db = I2C1_DR;
			sm_log_state(RECV_BYTE, 3, db);
			(*h->recv)(h->state, db);
		}
		/*
		 * XXX test vector, open address 2, write to it and see where
		 * the STOP ends up.
		 */
		sm_log_state(STOP, 0, 0);
		(*h->stop)(h->state, 0);
		/* Stop clearing sequence, read SR1 and write CR1 */
		sr1 = I2C_SR1(I2C1);
		I2C_CR1(I2C1) |= I2C_CR1_PE;
	}

	/*
	 * Handle the MIDDLE of an I2C transaction, moving DATA
	 *
	 * If we are receiving, read the next byte.
	 *
	 * If we are transmitting, send the next byte, how to stop
	 * gracefully is not yet well understood.
	 */
	if ((sr1 & I2C_SR1_BTF) != 0) {
		sr2 = I2C_SR2(I2C1);
		/* set which handle to use */
		h = ((sr2 & I2C_SR2_DUALF) == 0) ? dev[0] : dev[1];
		if (h == NULL) {
			/* handler not defined */
			/* XXX: do a STOP here and log error */
			return;
		}
		if ((sr1 & I2C_SR1_RxNE) != 0) {
			db = I2C1_DR;
			sm_log_state(RECV_BYTE, 4, db);
			(*h->recv)(h->state, db);
		} else if ((sr1 & I2C_SR1_TxE) != 0) {
			db = (*h->send)(h->state);
			sm_log_state(SENT_BYTE, 4, db);
			I2C1_DR = db;
		}
	}
}

void setup_i2c(uint8_t addr, i2c_handler_t *handle);

/*
 * Set up an I2C device as a slave with the passed in address.
 * We're going to use the pins on the Arduino connector labeled
 * as SCA/D14 and SCL/D15 which are associated with PB8 (SCA) and
 * PB9 (SCL).
 */
void
setup_i2c(uint8_t addr, i2c_handler_t *handle)
{
	int fpclk = rcc_apb1_frequency / 1000000;
	/*
 	 * Enable GPIOB (may be redundant but that is okay)
 	 * Set the pins to Alternate Function, low speed (2MHz), and open drain
 	 * as the I2C bus has pull-ups on the pins.
 	 */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ,
						GPIO8 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);

	/*
  	 * Enable the I2C1 peripheral (that is connected to these pins per
  	 * the CPU datasheet and Table 11. Alternate Function
  	 */
	rcc_periph_clock_enable(RCC_I2C1);
//	i2c_set_own_7bit_address(I2C1, addr);
	I2C_OAR1(I2C1) = addr << 1; /* Give ourselves an address */

	I2C_CCR(I2C1) = 0x8000 | (((fpclk * 5) / 6) & 0xfff);
	I2C_TRISE(I2C1) = (fpclk + 1) & 0x3f;
	/* This is required or we get bus errors intermittently */
	I2C_FLTR(I2C1) = 15; /* max filtering */
	
	/* enable interrupts for events, tell it APB1 is set to 42 MHz */
	I2C_CR2(I2C1) = I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | 42;
	/* enable interrupts from I2C */
	nvic_enable_irq(NVIC_I2C1_EV_IRQ);
	nvic_enable_irq(NVIC_I2C1_ER_IRQ);
	I2C_CR1(I2C1) = I2C_CR1_ACK | I2C_CR1_PE; /* turn on the peripheral */
	dev[0] = (i2c_handler_t *) malloc(sizeof(i2c_handler_t));
	if (dev[0] == NULL) {
		fprintf(stderr, "Unable to allocate handler\n");
		return;
	}
	memcpy(dev[0], handle, sizeof(i2c_handler_t));
	dev[0] = handle;
	dev[0]->addr = addr;
	dev[0]->mode = HANDLE_MODE_IDLE;
	dev[0]->state = (*dev[0]->init)();
	if (dev[0]->state == NULL) {
		fprintf(stderr, "Unable to initialize handler\n");
		return;
	}
}

void dump_status(uint16_t sr);

/*
 * Generalized dump of things I might find interesting while debugging
 */
void
dump_status(uint16_t sr)
{
	printf("I2C Status: 0x%04x\n", sr);
	printf("\t%s\n", ((sr & I2C_SR1_SMBALERT) != 0) ?
		(RED_TEXT "SMB alert") :
		(GREEN_TEXT "No SMB alert"));
	printf("\t%s\n", ((sr & I2C_SR1_TIMEOUT) != 0) ?
		(RED_TEXT "Timeout") :
		(GREEN_TEXT "No timeout"));
	printf("\t%s\n", ((sr & I2C_SR1_PECERR) != 0) ?
		(RED_TEXT "PEC error") :
		(GREEN_TEXT "No PEC error"));
	printf("\t%s\n", ((sr & I2C_SR1_OVR) != 0) ?
		(RED_TEXT "Over/Underrun error") :
		(GREEN_TEXT "No Over or Underrun error"));
	printf("\t%s\n", ((sr & I2C_SR1_AF) != 0) ?
		(RED_TEXT "Acknowledge failure") :
		(GREEN_TEXT "No acknowledge failure"));
	printf("\t%s\n", ((sr & I2C_SR1_ARLO) != 0) ?
		(RED_TEXT "Arbitration lost") :
		(GREEN_TEXT "no lost arbitration"));
	printf("\t%s\n", ((sr & I2C_SR1_BERR) != 0) ?
		(RED_TEXT "Bus error") :
		(GREEN_TEXT "No bus error"));
	printf("\t%s\n", ((sr & I2C_SR1_TxE) == 0) ?
		(GREEN_TEXT "Ready to transmit (Tx)") :
		(RED_TEXT "Data register not empty (Tx)"));
	printf("\t%s\n", ((sr & I2C_SR1_RxNE) == 0) ?
		(GREEN_TEXT "Ready to receive (Rx)") :
		(RED_TEXT "Data register not empty (Rx)"));
	printf("\t%s\n", ((sr & I2C_SR1_STOPF) != 0) ?
		(RED_TEXT "Stop condition detected") :
		(GREEN_TEXT "No stop condition detected"));
	printf("\t%s\n", ((sr & I2C_SR1_ADD10) != 0) ?
		(RED_TEXT "Master has sent first byte A10") :
		(GREEN_TEXT "No ADD10 event"));
	printf("\t%s\n", ((sr & I2C_SR1_BTF) != 0) ?
		(GREEN_TEXT "Data transfer done") : 
		(RED_TEXT "Data transfer NOT done"));
	printf("\t%s\n", ((sr & I2C_SR1_ADDR) != 0) ?
		(GREEN_TEXT "Received address MATCHED") : 
		(YELLOW_TEXT "Address mismatch or not received"));
	printf("\t%s\n", ((sr & I2C_SR1_SB) != 0) ?
		(GREEN_TEXT "Start condition generated") : 
		(RED_TEXT "No start condition"));
	printf("%s\n", DEFAULT_TEXT);
	sr = I2C_SR2(I2C1);
	printf("Secondary status:\n");
	printf("\t%s\n", ((sr & I2C_SR2_BUSY) == 0) ?
		(GREEN_TEXT "Bus idle") : 
		(RED_TEXT "Bus Busy"));
	printf("\t%s\n", ((sr & I2C_SR2_MSL) != 0) ?
		(RED_TEXT "Master mode") : 
		(GREEN_TEXT "Slave mode"));
	printf("\t%s\n", ((sr & I2C_SR2_TRA) != 0) ?
		(RED_TEXT "Transmit mode") : 
		(GREEN_TEXT "Receive mode"));
	printf("%s\n", DEFAULT_TEXT);
	if (logged_events() == 0) {
		printf("No interrupt events seen.\n");
	} else {
		printf("Interrupt events:\n");
		while (dump_event()) {
			;
		}
	}
}

void moveTo(int row, int col);

/*
 * Old school move the cursor around the screen
 * so that we can do a fancier display.
 */
void
moveTo(int row, int col)
{
	char	move_str[50];

	/* kind of an arbitrary limit */
	if ( ((row < 1) || (row > 50)) ||
		 ((col < 1) || (col > 150))) {
		return;
	}
	snprintf(move_str, 50, "\033[%d;%dH", row, col);
	uart_puts(move_str);
}

extern i2c_handler_t echo_handler;

int
main(void)
{
	uint32_t br, bs, er, es;
	uint8_t addr = 0x20;
	char buf[80];
	uint16_t membuf[MEM_REG_SIZE];
	nucleo_clock_setup();
	uart_setup(115200);
	led_setup();
	setup_i2c(addr, &echo_handler);
	memset(&mem_reg, 0, sizeof(mem_reg));
	memset(&reg_bank, 0, sizeof(reg_bank));
	/* initial so they will all display originally */
	for (int i = 0; i< 16; i++) {
		reg_bank[i].changed++;
	}
	mem_reg.changed++;
	moveTo(1,1);
	uart_puts(CLEAR_SCREEN);
	fprintf(stderr, "I2C Slave Test Program\n");
	printf("Address is set to 0x%2X hex or %d decimal\n", addr, addr);
	printf("Enter ^C at any time to restart.\n");
	while (1) {
		int test = 0;

		printf("Test Menu:\n");
		printf("\t1: Echo Test (echos back what is written)\n");
		printf("\t2: 'Register' - simulates 16 R/W 16 bit registers\n");
		printf("Enter test #: ");
		fflush(stdout);
		uart_gets(buf, 20);
		test = strtold((const char *)buf, NULL);
		switch (test) {
			case 1:
				br = bs = er = es = 0;
				printf("Waiting for data to be read or written\n");
				while (1) {
					if (1) {
						continue;
					}
					moveTo(1,1);
					uart_puts(CLEAR_LINE);
					printf("Sent: %15ld(%ld)\tReceived: %15ld(%ld)\n", 
												bs, es, br, er);
					uart_puts(CLEAR_LINE);
					while(dump_event()) {
						uart_puts(CLEAR_LINE);
					}
				}
				break;
			case 2:
				/* first lay out the form */
				moveTo(1,1);
				uart_puts(CLEAR_LINE);
				printf("Register State:\n");
				for (int i = 0; i < 8; i++) {
					moveTo(i + 2, 1);
					uart_puts(CLEAR_LINE);
					printf("R%02d =        (     )", i);
					printf("\033[5CR%02d =        (     )", i + 8);
					fflush(stdout);
				}
				moveTo(10, 1);
				uart_puts(CLEAR_LINE);
				moveTo(11, 1);
				uart_puts(CLEAR_LINE);
				uart_puts("Memory Register (R16) :");
				moveTo(12, 1);
				uart_puts(CLEAR_LINE);
				
				/*
				 * Now track register contents as they change
				 */
			 	while (1) {
					for (int i = 0; i < 16; i++) {
						if (reg_bank[i].changed) {
							moveTo((i / 2) + 2, (i & 1) * 25 + 7);
							printf("0x%04x", reg_bank[i].val);
							printf("\033[2C%5d", reg_bank[i].val);
							fflush(stdout);
							reg_bank[i].changed = 0;
						} 
					}
					/* flag memory locations that changed */
					if (mem_reg.changed) {
						moveTo(14, 1);
						mem_reg.changed = 0;
						for (int j = 0; j < MEM_REG_SIZE; j++) {
							membuf[j] = ((membuf[j] & 0xff) != mem_reg.buf[j]) ?
									membuf[j] | 0x100 : mem_reg.buf[j];
						}
						for (int j = 0; j < 2; j++) {
							uint8_t	*t = &(mem_reg.buf[j*16]);
							moveTo(13+j, 1);
							for (int k = 0; k < 16; k++) {
								if (membuf[j*2+k] & 0x100) {
									uart_puts(YELLOW_TEXT);
									membuf[j*2+k] &= 0xff;
								}
								printf("%02x ", *(t+k));
								fflush(stdout);
								uart_puts(DEFAULT_TEXT);
								if (k == 7) {
									uart_puts("  ");
								}
							}
							uart_puts("   ");
							for (int k = 0; k < 16; k++) {
								if (membuf[j*2+k] & 0x100) {
									uart_puts(YELLOW_TEXT);
								}
								uart_putc((isprint(*(t+k))) ? 
									(char) *(t+k) : '.');
								uart_puts(DEFAULT_TEXT);
							}
						}
					}
					msleep(100);
				}
				break;
				case 3:
					printf("\nDumping state tables on transactions:\n");
					while(1) {
						while (sm_cur_event == sm_nxt_event) {
							msleep(100);
						}
						sm_dump_state();
						printf("\n-------- separator -------\n");
					}
					break;
				default:
					printf("Unknown test\n");
					break;
			}
		}
}

