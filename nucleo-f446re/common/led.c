/*
 * Set up the LED on the Nucleo board to be driven on/off.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "../common.h"

void
led_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}

void
led_on(void)
{
	gpio_set(GPIOA, GPIO5);
}

void
led_off(void)
{
	gpio_clear(GPIOA, GPIO5);
}

int
led_toggle(void)
{
	gpio_toggle(GPIOA, GPIO5);
	/* return the previous state (inverse of the current state) */
	return (gpio_get(GPIOA, GPIO5) != 0) ? 0 : 1;
}
