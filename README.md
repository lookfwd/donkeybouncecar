# Donkeycar


## Todo

https://sketchfab.com/3d-models/red-dragon-ac2edeb01b9a4d4796c4d34f5d8aa1f2

https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black?view=all#software

https://learn.adafruit.com/assets/24667

https://github.com/lookfwd/donkeybouncecar



## Video

[![Watch the video](images/first-drive.png)](https://www.youtube.com/watch?v=y7wChCVcuVY)

## What

A shield that adds a GPS, a gyro, PWM monitors and a distance sensor

![](arduino/donkeybounceshield_bb.png)

The schematic

![](arduino/donkeybounceshield_schem.png)

The Shield

![](arduino/donkeybounceshield_pcb.png)

You can find details about the PCB [here](https://oshpark.com/shared_projects/IXQhfZmE).


## How to

```
ssh pi@192.168.1.124
```

If it's on `Ellipsis Jetpack 15BF`:

```
ssh pi@192.168.1.11
```


Wifi notes: Setting WiFi up via the command line
https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md


Ellipsis Jetpack 15BF


```
$ python a.py
```

This was a good attempt - but didn't work:

```
./usbrawserial /dev/ttyACM0
```


[Ref](http://docs.donkeycar.com/guide/get_driving/):
```
$ cd ~/mycar/
$ python manage.py drive
```

http://192.168.1.124:8887
http://192.168.1.11:8887

## GPS

* GPS Receiver - LS20031 5Hz ([1](https://www.sparkfun.com/products/8975), [2](https://learn.sparkfun.com/tutorials/ls20031-5hz-66-channel-gps-receiver-hookup-guide?_ga=2.230844831.1449235539.1558239626-1152025920.1554262143), [3](https://cdn.sparkfun.com/datasheets/GPS/LS20030~3_datasheet_v1.3.pdf))

## References

* [Pulse Width Modulation](https://learn.sparkfun.com/tutorials/pulse-width-modulation/all)
* [Arduino Uno SMD](https://www.arduino.cc/en/Main/ArduinoBoardUnoSMD)
* [SPI library](https://www.arduino.cc/en/reference/SPI)
* [Three Ways To Read A PWM Signal With Arduino](http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/)
* [MPU-6000 and MPU-6050 Register Map and Descriptions](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
* [MPU-6000 and MPU-6050 Product Specification](https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf)
* [MPU-6050 Accelerometer + Gyro](https://playground.arduino.cc/Main/MPU-6050/#short)
* [USB-ISS  Multifunction USB Communications Module - I2C Mode](https://www.robot-electronics.co.uk/htm/usb_iss_i2c_tech.htm)
* [USB-ISS  Multifunction USB Communications Module - I/O Mode](https://www.robot-electronics.co.uk/htm/usb_iss_io_tech.htm)
* [How to use the accelerometer- gyroscope GY-521](https://create.arduino.cc/projecthub/Nicholas_N/how-to-use-the-accelerometer-gyroscope-gy-521-6dfc19)
* [SparkFun FTDI Basic Breakout - 3.3V](https://www.sparkfun.com/products/9873)
* [pyserial/serial/serialposix.py](https://github.com/pyserial/pyserial/blob/master/serial/serialposix.py)
* [Make Raspberry Pi and Arduino Work as One Team](https://www.sunfounder.com/blog/rpi-ard/)
* [Interface a Raspberry Pi with an Arduino so the two boards can communicate with one another](https://maker.pro/raspberry-pi/tutorial/how-to-connect-and-interface-raspberry-pi-with-arduino)
* [TinyGPSPlus/examples/DeviceExample/DeviceExample.ino](https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino)
* [LS20031 5Hz (66 Channel) GPS Receiver Hookup Guide](https://learn.sparkfun.com/tutorials/ls20031-5hz-66-channel-gps-receiver-hookup-guide/all)