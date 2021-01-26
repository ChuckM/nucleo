/*
 * An example that uses the UART
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
#include "../common.h"

int
main(void)
{
	char	buf[80];
	int		len;

	/*
     * Clock setup must happen first so that the UART code can calculate
     * the correct divisor for the baudrate.
     */
	nucleo_clock_setup(1);
	/*
	 * Set up the the UART but note that not all ST-LINK versions can reliably
	 * do 115,200 baud, you may need to lower it to 57600 to get reliable
	 * operation if you are doing a lot of I/O to the serial port.
	 */
	uart_setup(115200);
	while (1) {
		uart_puts("Enter a string: ");
		len = uart_gets(buf, 80);
		/* if you want to kill the trailing newline you do it
		 * with the returned length value like so ...
		 */
		if (len > 0) {
			buf[len-1] = 0;
		}
		uart_puts("\n");
		uart_puts("You entered \"");
		uart_puts(buf);
		uart_puts("\"\n");
		/* The uart file also implements the stubs for stdio
		 * so you can use C functions like printf
		 */
		printf("This string was %d characters long.\n", len);
	}
}
