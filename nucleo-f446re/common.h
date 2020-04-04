/*
 * This is an include file that describes the functions available
 * in the common code. If you use them be sure to include the .o
 * files from the common code in your example.
 */
#pragma once
void nucleo_clock_setup(void);
void msleep(uint32_t delay);
uint32_t mtime(void);
unsigned char *time_string(uint32_t t);

void led_setup(void);
void led_on(void);
void led_off(void);
int	led_toggle(void);

