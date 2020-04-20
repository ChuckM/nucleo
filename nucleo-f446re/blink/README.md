# Blink Example #1 : Bare bones blinking

This is the simplest version in the family of Blink examples. It does nothing
but spin wait with NOPs to waste time. As a result it can be challenging to
have the LED blink at a particular rate.

The family of Blink examples are as follows:

  * **blink** -- This one, which uses a for loop with a nested NOP for its delay.
  * **blink-tick** -- Which takes advantage of the SysTick timer and interrupt
    service routine that is set up in the `clock.c`
  * **blink-timer** -- Which avoids the use of interrupts by using one of the
    basic timers available on the ST32 series to effectuate a specific delay.
  * **blink-helper** -- Which uses the available functions in the common code
    to both initialize the LED and to time the delay.

