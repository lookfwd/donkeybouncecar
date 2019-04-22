#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

// https://bit.ly/2IKISfB

void set_analog_mode(fd)
{
    unsigned char sbuf[4];
	sbuf[0] = 0x5A;							// USB_ISS command
	sbuf[1] = 0x02;							// Set mode
	sbuf[2] = 0x00;	   				// Set mode to I/O
	sbuf[3] = 0x03;							//  I/O1 Analog Input

	if (write(fd, sbuf, 4) < 0) perror("set_i2c_mode write");	// Write data to USB-ISS
	if (tcdrain(fd) < 0) perror("set_i2c_mode tcdrain");
	if (read(fd, sbuf, 2) != 2) perror("set_i2c_mode read");		// Read back error byte
	if(sbuf[0] != 0xFF)  // If first returned byte is not 0xFF then an error has occured
	{
		printf("**set_i2c_mode: Error setting I2C mode!**\n\n");
	}
}

int main(int argc, const char * argv[])
{
    if (argc < 1 || argc > 2)
    {
        fprintf(stderr, "usage: %s [device]", argv[0]);
        return 1;
    }

    const char* device = "/dev/cu.usbmodem000108531";
    if (argc >= 2) {
        device = argv[1];
    }

    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        perror("can't open device");
        exit(errno);
    }

    struct termios config;

	cfmakeraw(&config);			// make options for raw data
	if (tcsetattr(fd, TCSANOW, &config) < 0) {
        perror("tcsetattr config");   	// Set options for port
        exit(errno);
    }

    set_analog_mode(fd);

    int rc;

#define GETAD 0x65

    while (1) {
        unsigned char setup[] = {GETAD,    // Primary USB-ISS command
                                 1};       // Channel 1
        rc = write(fd, setup, sizeof setup);
        assert(rc==2);

    	if (tcdrain(fd) < 0) perror("tcdrain");

        unsigned char v[2];
    	if (read(fd, v, 2) != 2) {
            perror("read");		// Read back error byte
            exit(errno);
        }

        int vv = (v[0] << 8) + v[1];

        printf("%d\n", vv);

        //usleep(20000);
    }
    // while (1)
    // {
    //     char buffer[80];
    //     int n = read(fd, buffer, sizeof buffer);
    // }

    close(fd);
    return 0;
}
