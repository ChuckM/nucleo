# Programming ST Micro Nucleo Boards

[ST Micro](https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html)
makes an Arduino like board for many of their processors. These boards are
branded "Nucleo" and are available for $10 - $15 at distributors like
[Digikey](https://www.digikey.com/products/en/development-boards-kits-programmers/evaluation-boards-embedded-mcu-dsp/786?k=nucleo)
and
[Mouser](https://www.mouser.com/Embedded-Solutions/Engineering-Tools/Embedded-Processor-Development-Kits/Development-Boards-Kits-ARM/_/N-cxd2t?Keyword=nucleo&FS=True).

They have a couple of nice features, first they include a programming and debug
module that is "attached" and second they have an Arduino compatible set
of sheild pins which allows them to work with a number of pre-existing shields.
ST sells their own shields of course but there are not very many of them.

These boards support the [Mbed](https://os.mbed.com/) environment right out 
of the box and I recommend it for beginners. Some versions have
[Micropython](https://micropython.org/) ported to them 
(see here: https://micropython.org/stm32/) but getting it up and running is
not straight forward yet.

My preferred development environment is vim, make, and gdb. And doing native
or "bare metal", development on these boards can be straight forward. This
repository is a holder for my examples, write ups should appear at my
website : http://robotics.mcmanis.com/

