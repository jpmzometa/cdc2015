#!/bin/sh
UC=cm4
/home/chito/programs/stlink/st-flash write $UC/build/ch.bin 0x08000000
