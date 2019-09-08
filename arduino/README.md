# Donkey Bounce Shield II
## Goal
The second version of the Donkey Bounce Shield, uses a CPLD to accuratelly measure Servo signals from the Remote Control (RC). It creates PWM signals that drive both steering and throttle and by default it mirrors the signals of the RC. Alternatively, Arduino's serial port can override RC settings and drive the car. It also now supports various types of GPSs and both 6-axis and 9-axis absolute position sensors.

* You can find the PCB [here](https://oshpark.com/shared_projects/81d3538F).
* You can find the CPLD design and JTAG files [here](servo-reader-xc2c64a).
* You can find the latest Arduino Sketch [here](gps-gyro-pwm).

## Rationale

As soon as I started using the first version of the Donkey Bounce Shield, several limitations became aparent:

1. Measuring the value of Servo, created by the Remote Control, was highly inaccurate. Even after using interrupts and any other technique the accuracy was bad.
2. I could create PWM signals but I would have to manually unplug the Remote Control, when using them.
3. When, after several modifications I was able to have arduino as man-in-the-middle, Donkey Car was making random small moves as inaccurate measurements where propagated to the PWM controller.

The reason for inaccurate measurmeents was the sensitivity on interrupts. More specifically the `cli` command stops interrupts and it's used very often. For example, it's used by the `micros()` Arduino command and it's also very heavily used by the SoftwareSerial module. We need that module in order to read GPS signals.

Bottom line; In order to accurately read PWM signals you need to measure latencies with microsecond-level precision and an Arduino can't do that when it's as busy as it is in our case. I offloaded this work to [custom hardware and more specifically an XC2C64A Xilinx CPLD](servo-reader-xc2c64a).

## References

* [Use the Matrix Glitcher as a Coolrunner II XC2C64A development board](http://blog.dimitrioskouzisloukas.com/2019/08/use-matrix-glitcher-as-coolrunner-ii.html)
* [XC2C64A CoolRunner-II CPLD Datasheet](https://www.xilinx.com/support/documentation/data_sheets/ds311.pdf)
* [Datasheet of GPS smart antenna module, LS20030~3](https://cdn.sparkfun.com/datasheets/GPS/LS20030~3_datasheet_v1.3.pdf)
* [FGPMMOPA6H GPS Standalone Module Data Sheet](https://cdn-shop.adafruit.com/datasheets/GlobalTop-FGPMMOPA6H-Datasheet-V0A.pdf)
* [Adafruit Ultimate GPS v3 Schematic](https://learn.adafruit.com/adafruit-ultimate-gps/downloads)
* [Dangerous Prototypes XC2C64A cpld breakout Schematic](http://dangerousprototypes.com/docs/File:Cct-XC2C_64a-cpld-breakout-v1a.png)
* [TC1014/TC1015/TC1185 Voltage Regulators](http://ww1.microchip.com/downloads/en/devicedoc/21335e.pdf)
* [Logic Level Bidirectional Adaptor Schematic](https://cdn.sparkfun.com/datasheets/BreakoutBoards/Logic_Level_Bidirectional.pdf)
* [5V Tolerance Techniques for CoolRunner-II Devices](https://www.xilinx.com/support/documentation/application_notes/xapp429.pdf)
* [BSS138 N-Channel Logic Level Enhancement Mode Field Effect Transistor](https://cdn.sparkfun.com/datasheets/BreakoutBoards/BSS138.pdf)
* [Adafruit BNO055 Absolute Orientation Sensor](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor?view=all)
