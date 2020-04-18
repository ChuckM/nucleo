# I2C Slave Driver Example

This is an I2C slave driver. Full documentation will be on my web site shortly
but if you want to play with it connect to the nucleo's serial port with
an ANSI compatible terminal program (Hyperterm, Putty, GNU screen, Etc.)
and start the program. It implements two i2c addresses.

On address 32 is a simple "echo" handler. Bytes that are written to the device
are stored in a ring buffer, if you read from the device those same bytes are
returned. Until it runs out of bytes, then it returns `0xff`.

Meanwhile the screen will show you number of bytes sent, number of bytes
received, and any errors it got.

On address 42 is a emulated two line, 16 character display. It has seven (7)
"registers" four of which you can write too, two are read only, and one is 
write only.

## Operation

When the board is started it asks you which test to run. Test #1 runs the
echo handler and displays stats from that handler. Test #2 runs the display
handler and shows the emulated display. Test #3 just spits out I2C transactions
as they arrive. That is helpful for diagnosing problems with the master sending
crap.

Currently I'm using a Wemos D1 running Micropython to send i2c data to this
but will write  `master_i2c_example` that can drive it from another Nucleo
board. 

