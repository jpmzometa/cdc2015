#!/bin/sh
UC=cm4
/scratch/tmp/zmenende/mpc/uc/stlink/st-flash write $UC/build/ch.bin 0x08000000
