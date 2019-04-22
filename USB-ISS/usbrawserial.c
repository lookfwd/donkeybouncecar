#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <assert.h>


int main(int argc, const char * argv[])
{
    if (argc < 1 || argc > 2)
    {
        fprintf(stderr, "usage: %s [device]", argv[0]);
        return 1;
    }

    unsigned long bauds = 57600;
    
    const char* device = "/dev/cu.usbmodem14401";
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

    if (cfsetispeed(&config, bauds) < 0 || cfsetospeed(&config, bauds) < 0)
    {
        perror("can't set baud rate");
        exit(errno);
    }

	if (tcsetattr(fd, TCSANOW, &config) < 0) {
        perror("tcsetattr config");   	// Set options for port
        exit(errno);
    }

    while (1)
    {
        char buffer[1024];
        int n = read(fd, buffer, sizeof buffer);
        if (n >= 0)
            fwrite(buffer, n, 1, stdout);
    }

    close(fd);
    return 0;
}
