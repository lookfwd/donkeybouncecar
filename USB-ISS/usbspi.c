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

void display_version(fd)
{
    unsigned char sbuf[4];
	sbuf[0] = 0x5A; 						// USB_ISS byte
	sbuf[1] = 0x01;							// Software return byte

	if (write(fd, sbuf, 2) < 0) perror("display_version write");	// Write data to USB-ISS
	if (tcdrain(fd) < 0) perror("display_version tcdrain");
	if (read(fd, sbuf, 3) < 0) perror("display_version read");	// Read data back from USB-ISS, module ID and software version

	printf("USB-ISS Module ID: %u \n", sbuf[0]);
	printf("USB-ISS Software v: %u \n", sbuf[1]);
    printf("USB-ISS mode: %x \n\n", sbuf[2]);
}

void set_i2c_mode(fd)
{
    unsigned char sbuf[4];
	sbuf[0] = 0x5A;							// USB_ISS command
	sbuf[1] = 0x02;							// Set mode
	sbuf[2] = 0x60;	   				// Set mode to 100KHz I2C
	sbuf[3] = 0x00;							// Spare pins set to output low

	if (write(fd, sbuf, 4) < 0) perror("set_i2c_mode write");	// Write data to USB-ISS
	if (tcdrain(fd) < 0) perror("set_i2c_mode tcdrain");
	if (read(fd, sbuf, 2) != 2) perror("set_i2c_mode read");		// Read back error byte
	if(sbuf[0] != 0xFF)  // If first returned byte is not 0xFF then an error has occured
	{
		printf("**set_i2c_mode: Error setting I2C mode!**\n\n");
	}
}

void getAngle(double* pitch, double* roll, double x, double y, double z) {
    *pitch = atan(x/sqrt((y*y) + (z*z)));
    *roll = atan(y/sqrt((x*x) + (z*z)));
    //convert radians into degrees
    *pitch *= (180.0/3.14);
    *roll *= (180.0/3.14) ;
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
    // if (tcgetattr(fd, &config) < 0)
    // {
    //     perror("can't get serial attributes");
    //     exit(errno);
    // }

    // if (cfsetispeed(&config, bauds) < 0 || cfsetospeed(&config, bauds) < 0)
    // {
    //     perror("can't set baud rate");
    //     exit(errno);
    // }

    // config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK |
    //                     ISTRIP | IXON);
    // config.c_oflag = 0;
    // config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    // config.c_cflag &= ~(CSIZE | PARENB);
    // config.c_cflag |= CS8;
    // config.c_cc[VMIN]  = 1;
    // config.c_cc[VTIME] = 0;

	cfmakeraw(&config);			// make options for raw data
	if (tcsetattr(fd, TCSANOW, &config) < 0) {
        perror("tcsetattr config");   	// Set options for port
        exit(errno);
    }

    // if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
    // {
    //     perror("can't set serial attributes");
    //     exit(errno);
    // }

    display_version(fd);

    set_i2c_mode(fd);

#define I2C_AD1 0x55
#define MPU 0x68 << 1
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define READ 1

    int rc;

    // ------------------
    if (1) {
        unsigned char setup[] = {I2C_AD1,    // Primary USB-ISS command
                                 MPU,        // Device Address + R/W bit
                                 PWR_MGMT_1, // Device internal register
                                 1,          // Number of data bytes
                                 0};         // The data bytes
        rc = write(fd, setup, sizeof setup);
        printf("WAKE-UP: %d\n", rc);
    
    	if (tcdrain(fd) < 0) perror("set_i2c_mode tcdrain");

        unsigned char rv;
    	if (read(fd, &rv, 1) != 1) {
            perror("set_i2c_mode read");		// Read back error byte
            exit(errno);
        }
        printf("WAKE-UP: Write success: %d\n", rv);
    }

    // ------------------
    // while(1){
        // for (unsigned char i = 0x40; i < 0x70; ++i) {
    if (1) {
    unsigned char i = 0x74;
    unsigned char whoami[] = {I2C_AD1,      // Primary USB-ISS command
                              MPU | READ,   // Device Address + R/W bit
                              i,         // Device internal register
                              1};           // Number of data bytes
    int rrc = write(fd, whoami, sizeof whoami);
    printf("WHOAMI: %d\n", rrc);

	if (tcdrain(fd) < 0) perror("set_i2c_mode tcdrain");

    unsigned char trv;
    rrc = read(fd, &trv, 1);
    printf("WHOAMI: %d: rrv: %d value: %d\n", i, rrc, trv); // Expect:  0x68
    }
    // }
    // }

    // return 0;


    int tAcX = 0, tAcY = 0, tAcZ = 0;
    int tGyX = 0, tGyY = 0, tGyZ = 0;
    double tpitch = 0, troll = 0;
    while (1) {
        printf("\x1B[H");
        
        char measure[] = {I2C_AD1,      // Primary USB-ISS command
                          MPU | READ,   // Device Address + R/W bit
                          ACCEL_XOUT_H, // Device internal register
                          14};          // Number of data bytes
        rc = write(fd, measure, sizeof measure);
        printf("%d\n", rc);

    	if (tcdrain(fd) < 0) perror("set_i2c_mode tcdrain");

        char values[14];
        rc = read(fd, values, sizeof values);
        printf("READ VALUES: %d\n", rc);

        // Data correction
        int AcXoff = -950;
        int AcYoff = -300;
        int AcZoff = 0;
        int toff = -1600;
        int GyXoff = 480;
        int GyYoff = 170;
        int GyZoff = 210;
    
        int AcX = ((int)values[0] << 8 | values[1]) + AcXoff;
        int AcY = ((int)values[2] << 8 | values[3]) + AcYoff;
        int AcZ = ((int)values[4] << 8 | values[5]) + AcZoff;

        double tx = ((int)values[6] << 8 | values[7]) + toff;
        tx = tx/340 + 36.53;
        tx = (tx * 9/5) + 32;

        int GyX = ((int)values[8] << 8 | values[9]) + GyXoff;
        int GyY = ((int)values[10] << 8 | values[11]) + GyYoff;
        int GyZ = ((int)values[12] << 8 | values[13]) + GyZoff;

        //get pitch/roll
        double pitch, roll;
        getAngle(&pitch, &roll, AcX, AcY, AcZ);

        double s = 0.97;

        tAcX = s * tAcX + (1-s) * AcX;
        tAcY = s * tAcY + (1-s) * AcY;
        tAcZ = s * tAcZ + (1-s) * AcZ;        
        tGyX = s * tGyX + (1-s) * GyX;
        tGyY = s * tGyY + (1-s) * GyY;
        tGyZ = s * tGyZ + (1-s) * GyZ;
        
        tpitch = s * tpitch + (1-s) * pitch;
        troll = s * troll + (1-s) * roll;

        printf("Ac: %10d %10d %10d | %10d %10d %10d | %6.2f  %6.2f\n",
               tAcX, tAcY, tAcZ, tGyX, tGyY, tGyZ, pitch, roll);
        //break;
        usleep(20000);
    }
    // while (1)
    // {
    //     char buffer[80];
    //     int n = read(fd, buffer, sizeof buffer);
    // }

    close(fd);
    return 0;
}
