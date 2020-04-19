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
#include "emdisplay_handler.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#define RED_TEXT 		"\033[31;40m"
#define GREEN_TEXT		"\033[32;40m"
#define YELLOW_TEXT		"\033[33;40m"
#define DEFAULT_TEXT	"\033[0m"
#define CLEAR_LINE		"\033[2K"
#define CLEAR_SCREEN	"\033[2J"

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
	uint8_t addr1 = 32;
	uint8_t addr2 = 42;
	char buf[80];
	struct echo_state_t *echo;
	struct emd_state_t *emd;

	nucleo_clock_setup(1);
	uart_setup(115200);
	led_setup();
	echo = echo_init(256);
	emd = emd_init();
	/* Set up our I2C with two addresses */
	setup_i2c(addr1, &echo_handler, (void *)echo,
		      addr2, &emd_handler, (void *)emd);
	moveTo(1,1);
	uart_puts(CLEAR_SCREEN);
	fprintf(stderr, "I2C Slave Test Program\n");
	printf("Address is set to 0x%2X hex or %d decimal\n", addr1, addr1);
	printf("Second Address is set to 0x%2X hex or %d decimal\n", addr2, addr2);
	printf("Enter ^C at any time to restart.\n");
	while (1) {
		int test = 0;

		printf("Test Menu:\n");
		printf("\t1: Echo Test (echos back what is written)\n");
		printf("\t2: DisplayEMU - simulates a 16 x 2 character display\n");
		printf("\t3: Protocol - dumps protocol messages\n");
		printf("Enter test #: ");
		fflush(stdout);
		uart_gets(buf, 20);
		test = strtold((const char *)buf, NULL);
		switch (test) {
			case 1:
				uart_puts(CLEAR_SCREEN);
				moveTo(1,1);
				printf("Waiting for data to be read or written\n");
				while (1) {
					if (echo->total == echo->processed) {
						msleep(100);
						continue;
					}
					echo->processed = echo->total;
					moveTo(3,1);
					uart_puts(CLEAR_LINE);
					uart_puts("Echo handler:\n");
					uart_puts(CLEAR_LINE);
					printf("\tBytes Received: %15ld (%ld errors)\n", 
						echo->bytes_recv, echo->recv_errs);
					uart_puts(CLEAR_LINE);
					printf("\t    Bytes Sent: %15ld (%ld errors)\n",
						echo->bytes_sent, echo->send_errs);
				}
				break;
			case 2:
				/* first lay out the form */
				uart_puts(CLEAR_SCREEN);
				moveTo(1,1);
				printf("Emulated Display:\n");
			 	while (1) {
					while (emd->state_changed == 0) {
						msleep(100);
						if (sm_log_size()) {
							uart_puts("\033[J"); /* clear to end of screen */
							sm_dump_state();
							printf("\n");
						}
					}
					emd->state_changed = 0;
					moveTo(3, 1);
					sprintf(buf,"\033[3%d;4%dm", emd->fg, emd->bg);
					printf("Display Registers\n");
					printf("-----------------\n");
					printf("[0] Cmd\t\t%d\n", emd->cmd);
					printf("[1] Row\t\t%d\n", emd->row);
					printf("[2] MaxRow\t%d\n", emd->row_max);
					printf("[3] Col\t\t%d\n", emd->col);
					printf("[4] MaxCol\t%d\n", emd->col_max);
					printf("[5] FgColor\t%d\n", emd->fg);
					printf("[6] BgColor\t%d\n", emd->bg);
					moveTo(3, 27);
					uart_puts("Display Contents");
					moveTo(4, 25);
					uart_puts("--------------------");
					moveTo(6, 25);
					uart_puts("+------------------+\n");
					for (int i = 0; i < emd->row_max; i++) {
						moveTo(i+7, 25);
						uart_puts("| ");
						uart_puts(buf);
						for (int k = 0; k < emd->col_max; k++) {
							char x = *(((char *) emd->display)+
										               i * emd->col_max + k);
							uart_putc(isprint(x) ? x : ' ');
						}
						uart_puts(DEFAULT_TEXT);
						uart_puts(" |");
					}
					moveTo(7+emd->row_max, 25);
					uart_puts("+------------------+\n");
					moveTo(14,1);
					printf("I2C Transaction:\n");
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

