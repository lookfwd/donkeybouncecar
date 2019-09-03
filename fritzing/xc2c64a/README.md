# Make fritzing model

Based on `DSPIC30f4011` which also has package of tqfp44. I didn't have to change the pcb nor breadboard. I re-did the XML (`.fzp` file - using `make-fzp.py`) because it was messed up and also had to do serious modifications to `xc2c64a-schematic.svg` (using `make-schematic.py`) because OmniGraffle was creating very different `.svg` than expected and wasted my time.

Overall I followed the process described [here](https://www.deviceplus.com/how-tos/creating-custom-parts-fritzing/4/) and used as resources the [xilinx vq44 package spec](https://www.xilinx.com/support/documentation/package_specs/vq44.pdf), the [CoolRunner-II CPLD breakout board description page](http://dangerousprototypes.com/docs/CoolRunner-II_CPLD_breakout_board) and [shematic](http://dangerousprototypes.com/docs/images/e/e8/Cct-XC2C_64a-cpld-breakout-v1a.png), the [XC2C64A CoolRunner-II CPLD datasheet](https://www.xilinx.com/support/documentation/data_sheets/ds311.pdf), and the [seeedstudio page for the dev board](https://www.seeedstudio.com/XC2C64A-CoolRunner-II-CPLD-development-board-p-800.html).

