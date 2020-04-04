#include <stdint.h>
#include "../common.h"

int
main(void)
{
	nucleo_clock_setup();
	led_setup();
	while (1) {
		led_toggle();
		msleep(100);
	}
}
