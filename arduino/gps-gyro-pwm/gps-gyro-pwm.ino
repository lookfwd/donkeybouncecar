#include <SoftwareSerial.h>
#include <Wire.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 57600;

SoftwareSerial ss(RXPin, TXPin);

struct pos_t
{
//  float timeUtc;
  float lat;
  char latDir;
  float lon;
  char lonDir;
  int q;  // Zero => no signal. Note q=6 might not be good. Also check SV.
  int sv; // Number of Satellites in View
};

#define BUFSIZE 200
class NmeaProcessor
{
  char buf[BUFSIZE];
  int cnt;

public:
  char nmea[64];

  NmeaProcessor(): cnt(0) {
    buf[cnt] = 0;
  }

  bool parseNmea() {
    bool valid = false;
    if (strncmp("$GPGGA,", buf, 7)) {
      return false;
    }

    //offset of the 8th comma;
    char *secondCommaAt = 0, *eightCommaAt = 0;
    int comma = 0;
    for (char* p = buf; *p; ++p) {
      if (*p == ',') {
        ++comma;
        if (comma == 2) {
          secondCommaAt = p;
        }
        else if (comma == 8) {
          eightCommaAt = p;
          break;
        }
      }
    }
    if (eightCommaAt) {
      char* start = secondCommaAt + 1;
      int len = eightCommaAt - start;
      memcpy(nmea, start, len);
      nmea[len] = 0;
      valid = 1;
    }
    return valid;
  }

  bool addChar(char c) {
    if (cnt + 2 > BUFSIZE) {
      cnt = buf[0] = 0;
    }
    buf[cnt] = c;
    ++cnt;
    buf[cnt] = 0;

    if (buf[cnt-1] == '\n' && cnt > 1 && buf[cnt-2] == '\r') {

      buf[cnt-2] = 0; // Null terminate it
      
      bool valid = parseNmea();

      cnt = 0; // Reset
      buf[cnt] = 0;

      return valid;
    }
    return -1;
  }
};

NmeaProcessor np;
struct pos_t pos;

char n_cache[64];

const int MPU=0x68; 

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  Serial.begin(115200);           //  setup serial

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  ss.begin(GPSBaud);
  n_cache[0] = 0;
}


void loop() {
  //analog();
  
  while (ss.available() > 0) {
    if (!np.addChar(ss.read())) {
      strcpy(n_cache, np.nmea);
    }
  }
  
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  Wire.read();  // Tempeature H
  Wire.read();  // Tempeature L
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();

  int p1 = pulseIn(A3, HIGH);
  int p2 = pulseIn(A2, HIGH);
  char buf[100];
  sprintf(buf, "%5d|%5d|%d|%d|%d|%d|%d|%d|%s|",
          p1, p2,
          AcX, AcY, AcZ,
          GyX, GyY, GyZ,
          n_cache);
  Serial.println(buf);

  n_cache[0] = 0;
}
