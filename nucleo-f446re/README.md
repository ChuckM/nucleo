# NUCLEO-F446RE

This is the code for playing around with the
[NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)
board.

Things to note about this board:
  * The realtime clock crystal is fitted at X2 with a frequency of 32.768 kHz.
    This connects it to `PC14` and `PC15`.
  * The user LED is connected to `PA5`.
  * The serial port is connected to `PA2` and `PA3` (`USART2`).
  * The user button (blue) is connected to `PC13`.
  * The `OSC_IN` pin is fed by the 8 MHz crystal on the ST-Link portion.
  * This board runs ST-Link V2, is compatible with MBED, and exports three
    interfaces on the USB, Serial, Storage, and Debug.
  * It is supported by [OpenOCD](https://openocd.org) using
    the `st_nucleo_f4.cfg` file.

The common directory has clock, LED, and serial port set up code that is common
to various examples. 

By default, the clock code sets the processor to run from the PLL clock
generating a 168 MHz system clock (max). Which may not be what you want if
you are trying to use the USB-OTG peripheral (which needs a 48 MHz clock, and
there is no divider for 168 that gives you 48.)

See the individual example directories for specific information about that
example.

The Makefile infrastructure was derived from the original libopencm3 examples
directory. It still has remnants of that system in it.
