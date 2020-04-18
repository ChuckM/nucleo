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

#include <stdint.h>
#include <stdlib.h>	/* for malloc */
#include <string.h>	/* for memcpy */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "events.h"
#include "slave_i2c.h"

/*
 * One handler for each address
 */
static i2c_handler_t	*(dev[2]);
static i2c_handler_t	*__i2c_h;	/* cached pointer */

/*
 * Interrupt service routine for I2C interface #1 Errors
 */
void
i2c1_er_isr(void)
{
	uint16_t sr1;

	sr1 = I2C_SR1(I2C1);

	if ((sr1 & I2C_SR1_STOPF) != 0) {
		sm_log_state(STOP, 0, 0);
		I2C_CR1(I2C1) |= I2C_CR1_STOP;
		if ((sr1 & I2C_SR1_AF) != 0) {
			sm_log_state(NAK, 1, 0);
		}
		if (__i2c_h != NULL) {
			(*__i2c_h->stop)(__i2c_h->state, 0);
		}
	}
	if ((sr1 & I2C_SR1_TIMEOUT) != 0) {
		sm_log_state(ERROR, 0, SM_ERR_TIMEOUT);
		if (__i2c_h != NULL) {
			(*__i2c_h->stop)(__i2c_h->state, SM_ERR_TIMEOUT);
		}
	}
	if ((sr1 & I2C_SR1_OVR) != 0) {
		sm_log_state(ERROR, 1, SM_ERR_OVERUNDERFLOW);
		if (__i2c_h != NULL) {
			(*__i2c_h->stop)(__i2c_h->state, SM_ERR_OVERUNDERFLOW);
		}
	}

	/*
	 * This appears to be "NAK" which is literally "No Acknowledge"
	 * and this is described as "Set by hardware when no ack is returned."
	 */
	if ((sr1 & I2C_SR1_AF) != 0) {
		I2C_SR1(I2C1) = I2C_SR1(I2C1) & ~(I2C_SR1_AF);
		if (__i2c_h) {
			sm_log_state(NAK, 2, 0);
			(*__i2c_h->stop)(__i2c_h->state, 0);
		} else {
			sm_log_state(NAK, 3, 0);
		}
		/* stop after no acknowledge (NAK) */
		sm_log_state(STOP, 1, 0);
		I2C_CR1(I2C1) |= I2C_CR1_STOP;
	}
	/* Note this happens often if you don't have the noise filter set */
	if ((sr1 & I2C_SR1_BERR) != 0) {
		I2C_SR1(I2C1) = I2C_SR1(I2C1) & ~(I2C_SR1_BERR);
		sm_log_state(ERROR, 2, SM_ERR_BUSERROR);

		if (__i2c_h) {
			(*__i2c_h->stop)(__i2c_h->state, SM_ERR_BUSERROR);
		}
	}
}

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
 	 *
 	 * Note: Also this is the only time we know for sure which handler we
 	 * should be using so we cache that in __i2c_h for other code to use.
 	 * If __i2c_h is NULL then we don't have a handler attached.
 	 */
	if ((sr1 & I2C_SR1_ADDR) != 0) {
		i2c_event_t aa;
		uint8_t	addr = 0xff;

		/* This resets teh ADDR flag per the reference manual */
		sr2 = I2C_SR2(I2C1);

		/*
		 * Now, check to see if it is ADDR1 or ADDR2 (DUALF == 0 means ADDR1)
		 * then set which handler to use. (dev[0] for ADDR1, dev[1] for ADDR2)
		 * Note if its a read (we're sending) or a write (we're receiving)
		 * and put that into 'aa' for logging.
		 */
		if ((sr2 & I2C_SR2_DUALF) == 0) {
			__i2c_h = dev[0];
			addr = __i2c_h->addr;
			aa = ((sr2 & I2C_SR2_TRA) != 0) ? ADDR1_READ : ADDR1_WRITE ;
		} else {
			__i2c_h = dev[1];
			addr = __i2c_h->addr;
			aa = ((sr2 & I2C_SR2_TRA) != 0) ? ADDR2_READ : ADDR2_WRITE ;
		}

		if ((sr2 & I2C_SR2_TRA) != 0) {
			/*
			 * If the TRA bit is non zero we're transmitting 
			 * data *to* the master
			 */
			sm_log_state(aa, 1, addr);
			if (__i2c_h) {
				(*__i2c_h->start)(__i2c_h->state, HANDLE_MODE_SENDING);
			}

			/*
			 * DR starts empty, so we verify that and send first data
			 * byte.
			 */
			if (sr1 & I2C_SR1_TxE) {
				db = 0xff;
				if (__i2c_h) {
					db = (*__i2c_h->send)(__i2c_h->state);
				}
				sm_log_state(SENT_BYTE, 1, db); 
				I2C1_DR = db; /* send the actual byte */
			}
		} else {
			/*
			 * When the TRA bit is zero we're receiving
			 * data *from* the master
			 */
			sm_log_state(aa, 1, addr);
			if (__i2c_h) {
				(*__i2c_h->start)(__i2c_h->state, HANDLE_MODE_RECEIVING);
			}

			/*
			 * Now we have to wait for the first byte because it may be a
			 * one byte transfer and we won't see an ACK so we won't see
			 * and ACK Failure so we won't get a BTF event to pick it up.
			 */
			do {
				sr1 = I2C1_SR1;
			} while ((sr1 & (I2C_SR1_RxNE | I2C_SR1_TIMEOUT)) == 0);
			if ((sr1 & I2C_SR1_RxNE) != 0) {
				db = I2C1_DR;
				if (__i2c_h) {
					(*__i2c_h->recv)(__i2c_h->state, db);
					sm_log_state(RECV_BYTE, 1, db);
				}
			} else {
				if (__i2c_h) {
					(*__i2c_h->stop)(__i2c_h->state, SM_ERR_TIMEOUT);
				}
				sm_log_state(ERROR, 3, SM_ERR_TIMEOUT);
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
	 */
	if ((sr1 & I2C_SR1_STOPF) != 0) {
		sr2 = I2C_SR2(I2C1);
		if ((sr1 & I2C_SR1_RxNE) != 0) {
			db = I2C1_DR;
			if (__i2c_h) {
				(*__i2c_h->recv)(__i2c_h->state, db);
			}
			sm_log_state(RECV_BYTE, 2, db);
		}

		sm_log_state(STOP, 2, 0);
		if (__i2c_h) {
			(*__i2c_h->stop)(__i2c_h->state, 0);
		}
		__i2c_h = NULL;
		sr1 = I2C_SR1(I2C1);
		I2C_CR1(I2C1) |= I2C_CR1_PE;
	}

	/*
	 * Handle the MIDDLE of an I2C transaction, moving DATA
	 *
	 * If we are receiving, read the next byte.
	 *
	 */
	if ((sr1 & I2C_SR1_BTF) != 0) {
		sr2 = I2C_SR2(I2C1);
		if ((sr1 & I2C_SR1_RxNE) != 0) {
			db = I2C1_DR;
			if (__i2c_h) {
				(*__i2c_h->recv)(__i2c_h->state, db);
			}
			sm_log_state(RECV_BYTE, 3, db);
		} else if ((sr1 & I2C_SR1_TxE) != 0) {
			db = 0xff;
			if (__i2c_h) {
				db = (*__i2c_h->send)(__i2c_h->state);
			}
			sm_log_state(SENT_BYTE, 2, db);
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
int
setup_i2c(uint8_t addr1, const i2c_handler_t *handler1, void *handler1_state,
		  uint8_t addr2, const i2c_handler_t *handler2, void *handler2_state)
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
	/* Give ourselves a primary address */
	I2C_OAR1(I2C1) = addr1 << 1; 
	if (addr2 != 0) {
		/* Set second address and turn on ENDUAL */
		I2C_OAR2(I2C1) = addr2 << 1 | 1;
	}

	/* XXX master clock, not really needed by slave ? */
	I2C_CCR(I2C1) = 0x8000 | (((fpclk * 5) / 6) & 0xfff);
	I2C_TRISE(I2C1) = (fpclk + 1) & 0x3f;

	/* This is required or we get bus errors intermittently */
	I2C_FLTR(I2C1) = 15; /* max filtering */
	
	/* enable interrupts for events, tell it APB1 is set to 42 MHz */
	I2C_CR2(I2C1) = I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | 42;

	/*
	 * Set up the handler(s) that will respond to I2C messages
	 * coming in on this address.
	 *
	 * Note, you could use statically allocated handler and state
	 * structures if you want to avoid the use of malloc.
	 */
	dev[0] = (i2c_handler_t *) malloc(sizeof(i2c_handler_t));
	if (dev[0] == NULL) {
		return SI2C_RETURN_NO_MEMORY;
	}
	memcpy(dev[0], handler1, sizeof(i2c_handler_t));
	dev[0]->addr = addr1;
	dev[0]->state = handler1_state;
	if (dev[0]->state == NULL) {
		free(dev[0]);
		return SI2C_RETURN_MISSING_H1_STATE;
	}
	if (addr2 != 0) {
		dev[1] = (i2c_handler_t *) malloc(sizeof(i2c_handler_t));
		if (dev[1] == NULL) {
			free(dev[0]);
			return SI2C_RETURN_NO_MEMORY;
		}
		memcpy(dev[1], handler2, sizeof(i2c_handler_t));
		dev[1]->addr = addr2;
		dev[1]->state = handler2_state;
		if (dev[1] == NULL) {
			free(dev[1]);
			free(dev[0]);
			return SI2C_RETURN_MISSING_H2_STATE;
		}
	}

	/* enable interrupts from I2C */
	nvic_enable_irq(NVIC_I2C1_EV_IRQ);
	nvic_enable_irq(NVIC_I2C1_ER_IRQ);
	/* turn on the peripheral */
	I2C_CR1(I2C1) = I2C_CR1_ACK | I2C_CR1_PE; 
	return SI2C_RETURN_SUCCESS;
}

