# Common Code

This code provides a common set of APIs to all of the Nucleo projects. The
goal is to minimize the stuff you need to learn in order to get started
while maintaining the flexibility to go where ever your interest takes you.

First is the `clock.c` code. This code provides four external APIs:
  * `void nucleo_clock_setup(void)` -- This function initializes the
     stm32 clock tree to run the processor from the PLL clock at a
     much faster speed than the default.
  * `void msleep(uint32_t delay)` -- This function will delay code execution
     by `delay` milleseconds. Because it is impossible to know where in the
     current millisecond the code was called, the amount of delay for the
     for the first millesecond can be from about 50 microseconds to a full
     millesecond. More precise delays can be set up with dedicated timers.
  * `uint32_t mtime(void)` -- This function returns the value of a 
     monotonically increasing unsigned 32 bit integer. It is incremented
     1000 times per second.
  * `char * time_string(uint32_t timestamp)` -- This function converts an
     unsigned 32 bit value into a "time string" of the form: HHH:MM:SS.mmm
     where HHH is 0 to 999 hours, MM is 0 to 59 minutes, SS is 0 to 59
     seconds, and mmm is 0 to 999 milleseconds.

Next up is the `led.c` code. This code provides an API to manipulate the
on board user LED and provides five APIS:
  * `void setup_led(void)` -- Enable the GPIO that drives the LED and set it
    up as an output.
  * `void led_on(void)` -- Turn the on board LED on.
  * `void led_off(void)` -- Turn the on board LED off.
  * `int led_toggle(void)` -- Toggle the LED returning its previous state.
