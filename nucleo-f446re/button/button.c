/*
 * button - demonstrate use of the button
 *
 * This code counts button presses.
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
#include <stdlib.h>
#include "../common.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

void button_setup(void);

void
button_setup(void)
{
	/* enable clock to the GPIO with the button attached */
	rcc_periph_clock_enable(RCC_GPIOC);
	/* Configure the Button's GPIO pin as an input */
	/* XXX: do we need to set it as pullup? */
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO13);
	
	
}

int
main(void)
{
	int	button_count = 0;

	nucleo_clock_setup(1);
	uart_setup(115200);
	led_setup();
	button_setup();

	/* for debouncing, 1mS delay */
	rcc_periph_clock_enable(RCC_TIM6);
	TIM6_PSC = 83;
	TIM6_ARR = 9;
	TIM6_CR1 = TIM_CR1_OPM;
	

	printf("Button Example - polling\n");

	while (1) {
		/* spin loop waiting for a press */
		while (gpio_get(GPIOC, GPIO13)) ;
		TIM6_CR1 |= TIM_CR1_CEN;
		while (!(TIM6_SR & TIM_SR_UIF));
		/* didn't hold it long enough */
		if (gpio_get(GPIOC, GPIO13)) {
			continue;
		}
		uart_puts("Button pressed ... ");
		led_on();
		while (!(gpio_get(GPIOC, GPIO13))) ;
		TIM6_CR1 |= TIM_CR1_CEN;
		while (!(TIM6_SR & TIM_SR_UIF));
		uart_puts("and released. ");
		button_count++;
		printf("[ %d presses]\n", button_count);
		led_off();
	}	
}
