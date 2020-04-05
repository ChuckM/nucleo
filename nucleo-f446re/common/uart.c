/*
 * Helper functions for setting up the serial port that is
 * available through the Nucleo's USB interface.
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

/*
 * Console utility library - uses the serial port made available
 * by the debug port on the evaluation board as a default console.
 *
 * This version is interrupt driven.
 */

#include <stdint.h>
#include <ctype.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/cortex.h>
#include "../common.h"

/*
 * When you type ^C into the uart it will reset the system. Undefine
 * this if you want to be able to send ^C with out having this behavior.
 */
#define RESET_ON_CTRLC

/*
 * Some definitions of our uart "functions" attached to the
 * USART.
 *
 * These define sort of the minimum "library" of functions which
 * we can use on a serial port.
 */

#define UART_NVIC_IRQ	NVIC_USART2_IRQ

#define UART			USART2
#define UART_RCC		RCC_USART2
#define UART_GPIO		GPIOA
#define UART_GPIO_RCC	RCC_GPIOA
#define UART_TX			GPIO2
#define UART_RX			GPIO3

/*
 * This is a ring buffer to holding characters as they are typed
 * it maintains both the place to put the next character received
 * from the UART, and the place where the last character was
 * read by the program.
 */

#define RECV_BUF_SIZE	128		/* Arbitrary buffer size */
char recv_buf[RECV_BUF_SIZE];
volatile int recv_ndx_nxt;		/* Next place to store */
volatile int recv_ndx_cur;		/* Next place to read */

/* For interrupt handling we add a new function which is called
 * when recieve interrupts happen. The name (usart2_isr) is created
 * by the irq.json file in libopencm3 calling this interrupt for
 * USART2 'usart2', adding the suffix '_isr', and then weakly binding
 * it to the 'do nothing' interrupt function in vec.c.
 *
 * By defining it in this file the linker will override that weak
 * binding and instead bind it here, but you have to get the name
 * right or it won't work. And you'll wonder where your interrupts
 * are going.
 */
void
usart2_isr(void)
{
	uint32_t	reg;
	int			i;

	do {
		reg = USART_SR(UART);
		if (reg & USART_SR_RXNE) {
			recv_buf[recv_ndx_nxt] = USART_DR(UART);
#ifdef RESET_ON_CTRLC
			/*
			 * This bit of code will jump to the ResetHandler if you
			 * hit ^C
			 */
			if (recv_buf[recv_ndx_nxt] == '\003') {
				scb_reset_system();
				return; /* never actually reached */
			}
#endif
			/* Check for "overrun" */
			i = (recv_ndx_nxt + 1) % RECV_BUF_SIZE;
			if (i != recv_ndx_cur) {
				recv_ndx_nxt = i;
			}
		}
	/* can read back-to-back interrupts */
	} while ((reg & USART_SR_RXNE) != 0);
}

/*
 * uart_putc(char c)
 *
 * Send the character 'c' to the USART, wait for the USART
 * transmit buffer to be empty first.
 */
void
uart_putc(char c)
{
	uint32_t	reg;
	do {
		reg = USART_SR(UART);
	} while ((reg & USART_SR_TXE) == 0);
	USART_DR(UART) = (uint16_t) c & 0xff;
}

/*
 * char = uart_getc(int wait)
 *
 * Check the USART for a character. If the wait flag is
 * non-zero. Continue checking until a character is received
 * otherwise return 0 if called and no character was available.
 */
char
uart_getc(int wait)
{
	char		c = 0;

	while ((wait != 0) && (recv_ndx_cur == recv_ndx_nxt));
	if (recv_ndx_cur != recv_ndx_nxt) {
		c = recv_buf[recv_ndx_cur];
		recv_ndx_cur = (recv_ndx_cur + 1) % RECV_BUF_SIZE;
	}
	return c;
}

/*
 * void uart_puts(char *s)
 *
 * Send a string to the USART, one character at a time, return
 * after the last character, as indicated by a NUL character, is
 * reached.
 *
 * Translate '\n' in the string (newline) to \n\r (newline + 
 * carraige return)
 */
void
uart_puts(char *s)
{
	while (*s != '\000') {
		uart_putc(*s);
		/* Add in a carraige return, after sending line feed */
		if (*s == '\n') {
			uart_putc('\r');
		}
		s++;
	}
}

/*
 * int uart_gets(char *s, int len)
 *
 * Wait for a string to be entered on the console, with
 * support for editing characters (delete letter (^H or DEL),
 * word (^W),  entire line (^U)).
 *
 * It returns when the length is reached or a carrige return is entered,
 * which ever comes first. The <CR> character is changed to newline
 * before the buffer is returned.
 */
int
uart_gets(char *s, int len)
{
	char *t = s;
	char c;

	*t = '\000';
	/* read until a <CR> is received */
	while (((c = uart_getc(1)) != '\r') && ((t - s) < len) ) {
		if ((c == 0x8) || (c == 0x7f)) {
			if (t > s) {
				/* send ^H ^H to erase previous character */
				uart_puts("\010 \010");
				t--;
			}
		} else if (c == 0x17) {	// ^W erase a word
			while ((t > s) &&  (!(isspace((int) (*t))))) {
				t--;
				uart_puts("\010 \010");
			}
		} else if (c == 0x15) { // ^U erase the line
			while (t > s) {
				t--;
				uart_puts("\010 \010");
			}
		} else {
			*t = c;
			uart_putc(c);
			if ((t - s) < len) {
				t++;
			}
		}
		/* update end of string with NUL */
		*t = '\000';
	}
	if ((t < s) < len) {
		*t++ = '\n';
		*t = 0;
	}
	return t - s;
}

/*
 * Set up the GPIO subsystem with an "Alternate Function"
 * on some of the pins, in this case connected to a
 * USART.
 */
void
uart_setup(int baud)
{

	/* MUST enable the GPIO clock in ADDITION to the USART clock */
	rcc_periph_clock_enable(UART_RCC);
	rcc_periph_clock_enable(UART_GPIO_RCC);

	gpio_mode_setup(UART_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE,
			UART_TX | UART_RX);

	/* Actual Alternate function number (in this case 7) is part
	 * depenedent, check the data sheet for the right number to
	 * use.
	 */
	gpio_set_af(UART_GPIO, GPIO_AF7, UART_TX | UART_RX);

	recv_ndx_nxt = recv_ndx_cur = 0; /* initialize buffer ptrs */

	/* Set up USART/UART parameters using the libopencm3 helper functions */
	usart_set_baudrate(UART, baud);
	usart_set_databits(UART, 8);
	usart_set_stopbits(UART, USART_STOPBITS_1);
	usart_set_mode(UART, USART_MODE_TX_RX);
	usart_set_parity(UART, USART_PARITY_NONE);
	usart_set_flow_control(UART, USART_FLOWCONTROL_NONE);
	usart_enable(UART);

	/* Enable interrupts from the USART */
	nvic_enable_irq(UART_NVIC_IRQ);

	/* Specifically enable recieve interrupts */
	usart_enable_rx_interrupt(UART);
}

/*
 * Set a different baud rate for the console.
 */
void
uart_baud(int baud_rate)
{
	usart_set_baudrate(UART, baud_rate);
}

/*
 * These are the functions to define to enable the
 * newlib hooks to implement basic character I/O
 */
int _write (int fd, char *ptr, int len);
int _read (int fd, char *ptr, int len);

/*
 * A 128 byte buffer for getting a string from the
 * console.
 */
#define BUFLEN 128

static char buf[BUFLEN+1] = {0};
static char *next_char;

/* 
 * Called by libc stdio functions
 */
int 
_write (int fd, char *ptr, int len)
{
	int i = 0;

	/* 
	 * Write "len" of char from "ptr" to file id "fd"
	 * Return number of char written.
	 */
	if (fd > 2) {
		return -1;  // STDOUT, STDIN, STDERR
	}
	if (fd == 2) {
		/* set the text output YELLOW when sending to stderr */
		uart_puts("\033[33;40;1m");
	}
	while (*ptr && (i < len)) {
		uart_putc(*ptr);
		if (*ptr == '\n') {
			uart_putc('\r');
		}
		i++;
		ptr++;
	}
	if (fd == 2) {
		/* return text out to its default state */
		uart_puts("\033[0m");
	}
  return i;
}


/*
 * Depending on the implementation, this function can call
 * with a buffer length of 1 to 1024. However it does no
 * editing on console reading. So, the console_gets code 
 * implements a simple line editing input style.
 */
int
_read (int fd, char *ptr, int len)
{
	int	my_len;

	if (fd > 2) {
		return -1;
	}

	/* If not null we've got more characters to return */
	if (next_char == NULL) {
		uart_gets(buf, BUFLEN);
		next_char = &buf[0];
	}

	my_len = 0;
	while ((*next_char != 0) && (len > 0)) {
		*ptr++ = *next_char++;
		my_len++;
		len--;
	}
	if (*next_char == 0) {
		next_char = NULL;
	}
	return my_len; // return the length we got
}
