# Blink Example #2: Systick Based Blink

This example is the simplest example of using just the helper code
`nucleo_clock_setup` which sets the Nucleo board to run at 168 MHz
and sets up a recurring 1 kHz interrupt as the "system tick."

That interrupt enables us to write a simple delay function which is
quite precise. It takes the number of milleseconds we wish to sleep
and delays for at least that many milleseconds. (see the readme in
the common directory for more details). It will never sleep for more
than delay mS.

