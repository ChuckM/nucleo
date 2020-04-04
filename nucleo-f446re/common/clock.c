/*
 * Easier to use clock functions
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include "../common.h"

/*
 * Set of parameters to generate a 168 MHz system
 * clock on the STM32F446RE that is on this Nucleo board.
 *
 * You can put this chip into "overdrive" and run it at 180 MHz
 * but that is a bit more work than we need right now. Typically
 * 168 MHz is fine.
 */

static const struct rcc_clock_scale nucleo_446_clk_params = {
	.pllm = 4,
	.plln = 168,
	.pllp = 2,
	.pllq = 4, /* SDIO clock is 42 MHz */
	.pllr = 2,
	.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
	.ppre2 = 2,
	.ppre1 = 4,
	.hpre = 0,
	.voltage_scale = PWR_SCALE1,
	.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
					FLASH_ACR_LATENCY_5WS,
	.ahb_frequency = 168000000,
	.apb1_frequency = 42000000,
	.apb2_frequency = 84000000,
};

/* Set up a timer to create 1mS ticks. */
static void
systick_setup(int tick_rate)
{
    /* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(168000000 / tick_rate);
	systick_clear();
	systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();
}

/*
 * And this is the utility function, set up the clock for the
 * desired frequency and enable the SysTick counter to give
 * 1mS system "ticks" (1000 Hz).
 */
void
nucleo_clock_setup(void) {
	
	rcc_clock_setup_pll(&nucleo_446_clk_params);
	systick_setup(1000);
	return;
}

/*
 * monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 * declared volatile so that they don't get optimized out.
 */
static volatile uint32_t system_millis;

/*
 * non thread safe delay timer, set this to the number of milliseconds
 * to wait, and it gets decremented by the systick interrupt.
 */
static volatile uint32_t delay_millis;

/* Called when systick fires */
void
sys_tick_handler(void) {

    system_millis++;
	if (delay_millis) {
		delay_millis--;
	}
}

/* sleep for delay milliseconds */
void
msleep(uint32_t delay)
{
	delay_millis = delay;
    while (delay_millis) ;
}

/* return the time */
uint32_t
mtime()
{
    return system_millis;
}

/*
 * time_string(uint32_t)
 *
 * Convert a number representing milliseconds into a 'time' string
 * of HHH:MM:SS.mmm where HHH is hours, MM is minutes, SS is seconds
 * and .mmm is fractions of a second.
 *
 * Uses a static buffer (not multi-thread friendly)
 */
unsigned char *
time_string(uint32_t t)
{
    static unsigned char time_string[14];
    uint16_t msecs = t % 1000;
    uint8_t secs = (t / 1000) % 60;
    uint8_t mins = (t / 60000) % 60;
    uint16_t hrs = (t /3600000);

    // HH:MM:SS.mmm\0
    // 0123456789abc
    time_string[0] = (hrs / 100) % 10 + '0';
    time_string[1] = (hrs / 10) % 10 + '0';
    time_string[2] = hrs % 10 + '0';
    time_string[3] = ':';
    time_string[4] = (mins / 10)  % 10 + '0';
    time_string[5] = mins % 10 + '0';
    time_string[6] = ':';
    time_string[7] = (secs / 10)  % 10 + '0';
    time_string[8] = secs % 10 + '0';
    time_string[9] = '.';
    time_string[10] = (msecs / 100) % 10 + '0';
    time_string[11] = (msecs / 10) % 10 + '0';
    time_string[12] = msecs % 10 + '0';
    time_string[13] = 0;
    return &time_string[0];
}
