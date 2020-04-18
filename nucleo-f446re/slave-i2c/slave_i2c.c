/*
 * I2C Slave Device Driver
 *
 * This is an interrupt driven I2C driver that runs as a slave. Using it
 * consists of linking to this file and one or more handler files. The
 * interface is initialized with an address and an associated handler
 * and then started. 
 *
 * The handler will have a handler specific way of indicating that commands
 * are being received. See the echo_handler for an example.
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
#include "echo_handler.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

/*
 * One handler for each address
 */
static i2c_handler_t	*(dev[2]);

/*
 * Interrupt service routine for I2C interface #1 Errors
 */
void
i2c1_er_isr(void)
{
	i2c_handler_t	*h;
	uint16_t sr1, sr2;

	sr1 = I2C_SR1(I2C1);

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
		sr2 = I2C1_SR2;
		h = ((sr2 & I2C_SR2_DUALF) == 0) ? dev[0] : dev[1];
		if (h) {
			sm_log_state(NAK, 2, 0);
			(*h->stop)(h->state, 0);
		} else {
			sm_log_state(NAK, 3, 0);
		}
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

/* XXX: Determine if we need to keep this */
uint8_t	__addr = 0;

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
	 * the manual is unclear, it says DUALF is 'reset' during STOPF
	 * but is that before or after the read of SR2? And before or
	 * after resetting STOPF?
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
	 * XXX: This code assumes that DUALF remains in the "correct"
	 * state. We'll have to test that.
	 *
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


/*
 * Set up an I2C device as a slave with the passed in address.
 * We're going to use the pins on the Arduino connector labeled
 * as SCA/D14 and SCL/D15 which are associated with PB8 (SCA) and
 * PB9 (SCL).
 */
void
setup_i2c(uint8_t addr, const i2c_handler_t *handler, void *handler_state)
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
	memcpy(dev[0], handler, sizeof(i2c_handler_t));
	dev[0]->addr = addr;
	dev[0]->mode = HANDLE_MODE_IDLE;
	dev[0]->state = handler_state;
	if (dev[0]->state == NULL) {
		fprintf(stderr, "Missing handler state\n");
		return;
	}
}

