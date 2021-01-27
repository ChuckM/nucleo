/*
 * Timer based Blink
 *
 * This is another way to get a more precise time delay, using a
 * timer. 
 *
 * The STM32 chips have a number of timers associated with them from
 * simple to quite complex. ST tries to keep timers with the same name
 * on every chip to have the same characteristics. As a result you
 * can have a timer called "TIM6" (logically timer 6) which is on a
 * part with only 4 timers. 
 *
 * In this case TIM6, aka Timer 6, is a really simple implementation of
 * the timer and doesn't have the fancy capture and PWM features that
 * the more sophisticated timers have. That makes it idea for using as
 * a simple delay timer.
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
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "../common.h"

const char *apb1_periph[] = {
	"Timer 2",
	"Timer 3",
	"Timer 4",
	"Timer 5",
	"Timer 6",
	"Timer 7",
	"Timer 12",
	"Timer 13",
	"Timer 14",
	"Rsvd (9)",
	"Rsvd (10)",
	"Window Watchdog Clock",
	"Rsvd (12)",
	"Rsvd (13)",
	"SPI 2",
	"SPI 3",
	"Rsvd(16)",
	"USART 2",
	"USART 3",
	"UART 4",
	"UART 5",
	"I2C 1",
	"I2C 2",
	"USB", 
	"Rsvd(24)",
	"CAN Bus",
	"Rsvd(26)",
	"Backup",
	"Power",
	"DAC",
	"Rsvd(30)",
	"Rsvd(31)"
};

#define _RCC_REG(i) MMIO32(RCC_BASE + ((i) >> 5))
#define _RCC_BIT(i) (1 << ((i) & 0x1f))

static void bitset(char *name, enum rcc_periph_clken r) {
	printf("%s is reg 0x%08lx, bit %d\n", name, _RCC_REG(r), _RCC_BIT(r));
}


int
main(void)
{
	/*
	 * Set the clock rate to 168 MHz (see ../common/clock.c)
	 * This also sets the precalers for APB1 (the "slow" peripheral
	 * bus) and APB2 (the "fast" peripheral bus). In this case:
	 *     APB1 Clock Rate - 42 MHz
	 *     APB2 Clock Rate - 84 MHz
	 *
	 * Also, we don't bother enabling SysTick since we're using the
	 * timer for our time delay.
	 */
	nucleo_clock_setup(1);
	uart_setup(115200);
	printf("1] RCC_APB2ENR = 0x%8lx\n", RCC_APB2ENR);
	printf("   RCC_APB1ENR = 0x%8lx\n", RCC_APB1ENR);
	rcc_periph_clock_enable(RCC_GPIOC);
	printf("2] RCC_APB2ENR = 0x%8lx\n", RCC_APB2ENR);
	printf("   RCC_APB1ENR = 0x%8lx\n", RCC_APB1ENR);
	/* Set up our on board LED */
	led_setup();
	printf("3] RCC_APB2ENR = 0x%8lx\n", RCC_APB2ENR);
	printf("   RCC_APB1ENR = 0x%8lx\n", RCC_APB1ENR);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 
					GPIO2);
	gpio_clear(GPIOC, GPIO2);

	/* Enable our timer */
	rcc_periph_clock_enable(RCC_TIM6);
	printf("3] RCC_APB2ENR = 0x%8lx\n", RCC_APB2ENR);
	printf("   RCC_APB1ENR = 0x%8lx\n", RCC_APB1ENR);

	printf("TIM4 is reg 0x%08lx, bit %d\n", 
				_RCC_REG(RCC_TIM4), _RCC_BIT(RCC_TIM4));
	printf("TIM5 is reg 0x%08lx, bit %d\n", 
				_RCC_REG(RCC_TIM5), _RCC_BIT(RCC_TIM5));
	printf("Attempting to enable Timer #6 which is bit 0x%x\n", RCC_TIM6);
	while (1) {
		uint32_t reg;
		bitset("TIM2", RCC_TIM2);
		bitset("TIM3", RCC_TIM3);
		bitset("TIM4", RCC_TIM4);
		bitset("TIM5", RCC_TIM5);
		bitset("TIM6", RCC_TIM6);
		bitset("TIM7", RCC_TIM7);
		bitset("TIM12", RCC_TIM12);
		bitset("TIM13", RCC_TIM13);
		bitset("SPI2", RCC_SPI2);
		bitset("SPI3", RCC_SPI3);
		rcc_periph_clock_enable(RCC_TIM2);
		rcc_periph_clock_enable(RCC_TIM3);
		rcc_periph_clock_enable(RCC_TIM4);
		rcc_periph_clock_enable(RCC_TIM5);
		rcc_periph_clock_enable(RCC_TIM6);
		rcc_periph_clock_enable(RCC_TIM7);
		rcc_periph_clock_enable(RCC_TIM12);
		rcc_periph_clock_enable(RCC_TIM13);
		rcc_periph_clock_enable(RCC_SPI2);
		rcc_periph_clock_enable(RCC_SPI3);
		reg = RCC_APB1ENR;
		printf("  Current RCC_APB1ENR = 0x%08lx\n", reg);
		for (int i = 0 ; i < 30; i++) {
			if (reg & (1 << i)) {
				printf("\t%s ENABLED\n", apb1_periph[i]);
			}
		}
		if (RCC_APB1ENR & 0x10) {
			break;
		}
		msleep(1000);
	}

	/*
	 * The timer clocks are generated by the peripheral bus they are attached
	 * to by default. If the prescaler is set to anything other than 1,
	 * the timer clock is 2x the APB clock.
	 * Timer 6 is attached to APB1 so its clocked internally at 2 * 36 MHz or
	 * 72 MHz. 
	 *
	 * The timer clock goes into a prescaler (TIM6_PSC) which is a 'divide
	 * by n' where n is (PSC value + 1) (so if the PSC has 0 in it, which
	 * is the default, it divides by (0 + 1) or 1. Using the prescaler,
	 * we can set the interval between counts. The PSC is a 16 bit unsigned
	 * value so the maximum prescale value is 65,536 (65,535 + 1).
	 *
	 * For example, setting PSC to 71, equivalent to divide by 72, the
	 * timer will divide the 72 MHz clock to a 1 MHz clock. Thus each count
	 * in the timer represents 1 uS in time. Alternatively, setting it to 7199
	 * means the clock is 10 kHz so each count represents 100 uS.
	 *
	 * Timer 6's count is loaded into the ARR register. For timer 6, it only
	 * counts up (other counters have more flexible counting schemes).
	 *
	 * It is important to note that the UIF flag is set on *overflow*. What
	 * that means for this code is that the counter goes up to the count loaded
	 * into ARR, and then ONE MORE COUNT before it overflows. So one way to
	 * think of that is that the ARR register is one less than the actual
	 * delay. 
	 *
	 * Timer 6's prescaler and count are both 16 bit values, the timer counts
	 * up, so setting the count (ARR) to 0xffff means it will count all the
	 * way to 65,535 and then one more count to overflow. Thus the maximum
	 * delay you could create in this situation is 65536 / 1281.7383 or 51.1
	 * seconds.
	 *
	 * Some chips have Timer 5 and Timer 2 which are 32 bit counters and
	 * can give you much longer delays.
	 */
	TIM6_PSC = 7199; 		/* Each 'tick' is 100 uS */
	while (1) {
		TIM6_ARR = 499; 	/* (499 + 1) * .100 mS = 50 mS */
		led_toggle();		/* Alternates every loop: blinks rate 10 Hz */
		/* clear status register */
		TIM6_SR = 0;
		/* start timer in 'one shot' (OPM) counting up to 1000. */
		TIM6_CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
		if ((TIM_CR1(TIM6) & 1) == 0) {
			while (1) ;
		}
		/* We're done when the event flag goes true */
		gpio_toggle(GPIOC, GPIO2);
#if 1
		while (!(TIM6_SR & TIM_SR_UIF));
#else
		while (TIM6_CNT);
#endif
	}
}
