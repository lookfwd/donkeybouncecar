import serial
import sys

port = "/dev/ttyACM0"

s1 = serial.Serial(port, 115200)
s1.flushInput()

while True:
    if s1.inWaiting() > 0:
        inputValue = s1.read(1)
        sys.stdout.write(inputValue.decode('ascii'))
