#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <assert.h>

struct pos_t
{
    float timeUtc;
    float lat;
    char latDir;
    float lon;
    char lonDir;
    int q;  // Zero => no signal. Note q=6 might not be good. Also check SV.
    int sv; // Number of Satellites in View
};

int processToken(struct pos_t *pos, int token, const char *msg, int n)
{
    switch (token) {
        case 0: return strncmp("$GPGGA", msg, 6);
        case 1: return sscanf(msg, "%f", &pos->timeUtc);
        case 2: return sscanf(msg, "%f", &pos->lat) & 0;
        case 3: return sscanf(msg, "%c", &pos->latDir);
        case 4: return sscanf(msg, "%f", &pos->lon);
        case 5: return sscanf(msg, "%c", &pos->lonDir);
        case 6: return sscanf(msg, "%d", &pos->q);
        case 7: return sscanf(msg, "%d", &pos->sv);
    }
    return 0;
}

int parseNmea(struct pos_t *pos, char* message)
{
    pos->q = 0; // Mark invalid

    int last = 0, token = 0;
    for (int i = 0; i < (strlen(message) + 1); ++i) {
        if (message[i] == ',' || message[i] == '\0') {
            int rv = processToken(pos, token, message + last, i - last);
            if (token == 0 && rv != 0) {
                return -1; // Not "$GPGGA"
            }
            last = i + 1;
            token += 1;
        }
    }
    return token == 15 ? 0 : -1;
}

void shiftLeft(char *buf, int *cnt, int toRemove)
{
    assert(*cnt >= toRemove);
    memmove(buf, buf + toRemove, *cnt - toRemove);
    *cnt -= toRemove;
}


int main(int argc, const char * argv[])
{
    if (argc < 1 || argc > 3)
    {
        fprintf(stderr, "usage: %s [device] [bauds]", argv[0]);
        return 1;
    }

    const char* device = "/dev/cu.usbserial-A506P6MN";
    if (argc >= 2) {
        device = argv[1];
    }

    unsigned long bauds = 57600;
    if (argc == 3)
    {
        char* result;
        bauds = strtoul(argv[2], &result, 10);
        if (*result != '\0')
        {
            fprintf(stderr, "usage: %s device [bauds]", argv[0]);
            return 1;
        }
    }

    int fd = open(device, O_RDWR | O_NDELAY | O_NOCTTY);
    if (fd == -1)
    {
        perror("can't open device");
        exit(errno);
    }

    struct termios config;
    if (tcgetattr(fd, &config) < 0)
    {
        perror("can't get serial attributes");
        exit(errno);
    }

    if (cfsetispeed(&config, bauds) < 0 || cfsetospeed(&config, bauds) < 0)
    {
        perror("can't set baud rate");
        exit(errno);
    }

    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK |
                        ISTRIP | IXON);
    config.c_oflag = 0;
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        perror("can't set serial attributes");
        exit(errno);
    }

    clock_t lastTime = clock();
#define BUFSIZE 1024
    char buf[BUFSIZE];
    int cnt = 0;
    buf[cnt] = 0;

    struct pos_t pos;
    while (1)
    {
        char buffer[80];
        int n = read(fd, buffer, sizeof buffer);
        if (n >= 0) {
            int newEnd = cnt + n + 1;
            if (newEnd > BUFSIZE) {
                shiftLeft(buf, &cnt, newEnd - BUFSIZE);
            }
            memcpy(buf + cnt, buffer, n);
            cnt += n;
            buf[cnt] = 0;

            int foundLine;
            do {
                foundLine = 0;
                for (char* p = buf; *p; ++p) {
                    if (*p == '\n' && p > buf && *(p-1) == '\r') {
                        foundLine = 1;

                        *(p - 1) = 0; // Null terminate it

                        if (!parseNmea(&pos, buf)) {
                            clock_t now = clock();
                            float tt = (float)(now - lastTime) / CLOCKS_PER_SEC;
                            printf("delta: %5.4f - time: %f lat: %f %c lon: "
                                   "%f %c q: %d sv: %d\n",
                                   tt, pos.timeUtc, pos.lat, pos.latDir,
                                   pos.lon, pos.lonDir, pos.q, pos.sv);
                            lastTime = now;
                        }

                        // printf("Line: %s\n", buf);

                        shiftLeft(buf, &cnt, (p + 1) - buf);

                        break;
                    }
                }
            }
            while (foundLine);

            //
            // extractTotal(&total, [&lastTime, &pos](const string& s){

            // });
        }
    }

    close(fd);
    return 0;
}
