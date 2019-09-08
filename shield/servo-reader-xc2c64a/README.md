# Servo Reader for XC2C64A-CP56

This is a Verilog design using Xilinx ISE WebPACK. Contains the following:

## Design

* [servocount1.v](servocount1.v) - Single PWM reader
* [servocount.v](servocount.v) - Top-Level two-channel PWM reader

## Verification

* [servocount_tb.v](servocount_tb.v) - Testbench
* [wave.do](wave.do) - ModelSim simulation layout

## Constraints

* [servocount-cp56.ucf](servocount-cp56.ucf) Constraints for XC2C64A-CP56. It won't work for other package types for the same CPLD

## JTAG files

* [servocount-cp56.jed](servocount-cp56.jed) - JEDEC file for package CP56
* [servocount-cp56.xsvf](servocount-cp56.xsvf) - Scan file for package CP56
