#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "../common.h"

int
main(void)
{
	nucleo_clock_setup();
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	while (1) {
		gpio_toggle(GPIOA, GPIO5);
		msleep(250);
	}
}
