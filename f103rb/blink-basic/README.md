# Blink Example : Bare bones blinking

This is the simplest version in the family of Blink examples. It does nothing
but spin wait with NOPs to waste time. As a result it can be challenging to
have the LED blink at a particular rate.

While not particularly useful in its own right, this example is an excellent
way to prove to yourself that you can successfully use a set of tools to
build a program, link it against the libopencm3 library,  and program it into
the flash. There is very little about this program that can go wrong
with respect to the programs logical flow.

The family of Blink examples are as follows:

  * **blink-basic** -- This one, which uses a for loop with a nested NOP for
    its delay.
  * **blink-tick** -- Which takes advantage of the SysTick timer and interrupt
    service routine that is set up in the `clock.c`
  * **blink-timer** -- Which avoids the use of interrupts by using one of the
    basic timers available on the ST32 series to effectuate a specific delay.
  * **blink-helper** -- Which uses the available functions in the common code
    to both initialize the LED and to time the delay.

