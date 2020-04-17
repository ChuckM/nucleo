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
#include "echo_handler.h"
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

int
main(void)
{
	uint32_t br, bs, er, es;
	uint8_t addr = 0x20;
	char buf[80];
	uint16_t membuf[MEM_REG_SIZE];
	struct echo_state_t *echo;

	nucleo_clock_setup();
	uart_setup(115200);
	led_setup();
	echo = echo_init(256);
	setup_i2c(addr, &echo_handler, (void *)echo);
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
						while (sm_log_size() == 0) {
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

