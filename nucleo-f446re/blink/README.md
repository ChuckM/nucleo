# Blink Example #1 : Bare bones blinking

This is the simplest Blink example. It is one of a family of examples
starting with this one, which assumes no other code, to `blink_tick` which
assumes that you have set up the `SysTick` interrupt to generate a 1 KHz
system tick, and finally `blink_helper` which assumes you are using both
the clock setup and LED setup code (which abstracts away all libopencm3 code
in the example.)

