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

The `uart.c` code implements the UART API. This code configures the on board
USART so that you can communicate over the serial port that shows up when 
you plug in the Nucleo board. I presents the following APIS:
  * `void uart_setup(uint32_t baudrate)` -- This initializes the peripherals
    used by the Nucleo board, configures the pins, and sets the baudrate for
    8 bit, no parity, baudrate of your choosing communication. 

    Note that if you are using `nucleo_clock_setup`, you must call that
    _before_ you call `uart_setup` as it will change the clock speeds used
    to calculate the baudrate delay constant.
  * `void uart_baud(int baudrate)` -- This function changes the baudrate for
    the UART.
  * `void uart_putc(char c)` -- Send a character out to the UART, this waits
    for the UART to be available so it will block if another character is
    already being sent.
  * `char *uart_getc(int wait)` -- This retrieves a character from the UART.
    If the value `wait` is non-zero, the code blocks until a character is
    available. When `wait` is zero the code will return immediately returning
    a value of 0 if no character was available.
  * `void uart_puts(char *message)` -- This call will seen the NUL terminated
    string `message` out to the UART.
  * `int uart_gets(char *buf, int len)` -- This call will collect a string
    from the UART. It implements simple editing options and will return when
    it reads a <CR> character (ASCII \\r). It will also return if the buffer
    length is reached to avoid overwriting memory past the buffer.
  * `_read()` -- This is a newlib stub that is called by the standard i/o
    functions in the C library.
  * `_write()` -- This is a newlib stub that is called by the standard I/O
    functions in the C library.
