// TODO:
// - Make sure CPLDs are with weak high-impedence
// - Make sure you tie all I/O to the ground
// - Make sure PCB has all ground on the lower layer
// - Add optional resistors to PCB
// - Add the 3 alternative GPS solutions
// - Prototype both power supply and bare-n-mosfet Volt adapters
// - Add JTAG pins to PCB
// - Make sure I export/save the GPS component

#define ENABLE_GYRO
#define ENABLE_GYRO2
#define ENABLE_GPS
#define ENABLE_SONAR
#define ENABLE_PWM

#ifdef ENABLE_SONAR
#include <NewPing.h>
NewPing sonar(A1, A0);
#endif  // ENABLE_SONAR

#ifdef ENABLE_GYRO2
#include <Wire.h>
#endif  // ENABLE_GYRO2

#define SERVO_RESET 1500


//---------------------------------------------------------------------

#ifdef ENABLE_PWM
struct ServoValue
{
  char status;
  uint16_t timeUs;
};

class ServoReader
{
    const int PIN_CLK = 11;
    const int PIN_SCAN_EN = 13;
    const int PIN_SCAN_IN = 12;
    const char *STATES = "WRDO";  // Wait, Running, Done, Overflow

    static void clkStart()
    {
      // PCINT3/OC2A/MOSI <-> PB3 <-> Arduino Pin 11
      TCCR2A = TCCR2A & ~_BV(COM2A1) | _BV(COM2A0);
    }

    static void clkStop()
    {
      // PCINT3/OC2A/MOSI <-> PB3 <-> Arduino Pin 11
      TCCR2A = TCCR2A & ~_BV(COM2A1) & ~_BV(COM2A0);
    }

    static void clkLow()
    {
      asm("cbi %0, %1 \n"
          : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB3)
         );
    }

    static void tick()
    {
      asm("sbi %0, %1 \n"
          "cbi %0, %1 \n"

          : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB3)
         );
    }

    static void scanHigh()
    {
      asm("sbi %0, %1 \n"
          : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB5)
         );
    }

    static void scanLow()
    {
      asm("cbi %0, %1 \n"
          : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB5)
         );
    }

  public:

    void init()
    {
      if (digitalPinToPort(PIN_CLK) != 2) {
        Serial.println("Expected clock to be in PORTB");
        while (1);
      }
      if (digitalPinToPort(PIN_SCAN_IN) != 2) {
        Serial.println("Expected scan_out to be in PORTB");
        while (1);
      }
      if (digitalPinToPort(PIN_SCAN_EN) != 2) {
        Serial.println("Expected scan_en to be in PORTB");
        while (1);
      }
      if (digitalPinToBitMask(PIN_CLK) != 8) {
        Serial.println("Expected mask(clock) to be 8");
        while (1);
      }
      if (digitalPinToBitMask(PIN_SCAN_IN) != 16) {
        Serial.println("Expected mask(scan_out) to be 16");
        while (1);
      }
      if (digitalPinToBitMask(PIN_SCAN_EN) != 32) {
        Serial.println("Expected mask(scan_en) to be 16");
        while (1);
      }

      OCR2A = 0x00; // This gives 1MHz given prescaler of 8
      TCCR2A = _BV(WGM21) | ~_BV(WGM20); //  compare output mode bits to toggle mode (COM2A1:0 = 1)
      TCCR2B = _BV(CS21); // Prescaler clk/8

      clkLow();

      pinMode(PIN_CLK, OUTPUT);
      pinMode(PIN_SCAN_EN, OUTPUT);
      pinMode(PIN_SCAN_IN, INPUT);

      ServoValue sv[2];
      read(sv);  // At the end of read(), clock is (re)started
    }

    void read(ServoValue *sv)
    {
      // 0. See "Arduino Inline Assembly" book
      // 1. See also here: https://www.arduino.cc/en/Reference/PortManipulation
      // 2. $ export PATH=$PATH:/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/
      // 3. avr-objdump -S /var/folders/p7/xkm56c997zj9cj4rpx35rc2r0000gn/T/arduino_build_705153/read-servo-cpld.ino.elf > read-servo-cpld.txt

      uint8_t s0h = 0, s0l = 0;
      uint8_t s1h = 0, s1l = 0;

      clkLow(); // Stop the clock and start the scan
      clkStop();
      scanHigh();

      uint8_t oldSREG = SREG;  // Clear interrupts. Stop the world!
      cli();

      uint8_t tmp = PORTB;
      uint8_t bset = tmp | (1 << PORTB3);
      uint8_t bclear = tmp & ~(1 << PORTB3);

#define readDigit(r) __asm__ __volatile__ (  \
    "add %0, %0  \n"                           \
    "sbic %1, %2 \n"                           \
    "ori %0, 1 \n"                             \
    "out %3, %4  \n"                           \
    "out %3, %5  \n"                           \
    : "+r" (r)                                 \
    : "I" (_SFR_IO_ADDR(PINB)), "I" (PINB4),   \
    "I" (_SFR_IO_ADDR(PORTB)),               \
    "r" (bset), "r" (bclear)                 \
                                          );

      readDigit(s0h);
      readDigit(s0h);
      readDigit(s0h);
      readDigit(s0h);
      readDigit(s0h);
      readDigit(s0h);

      readDigit(s0l);
      readDigit(s0l);
      readDigit(s0l);
      readDigit(s0l);
      readDigit(s0l);
      readDigit(s0l);
      readDigit(s0l);
      readDigit(s0l);

      readDigit(s1h);
      readDigit(s1h);
      readDigit(s1h);
      readDigit(s1h);
      readDigit(s1h);
      readDigit(s1h);

      readDigit(s1l);
      readDigit(s1l);
      readDigit(s1l);
      readDigit(s1l);
      readDigit(s1l);
      readDigit(s1l);
      readDigit(s1l);
      readDigit(s1l);

      SREG = oldSREG;  // 9.5us later, we're done! Restore.
      // (28 x 5 clocks per loop + extras) / 16MHz

      scanLow();
      clkStart();

      sv[0].status = STATES[s0h >> 4];
      sv[0].timeUs = ((s0h & 0xf) << 8) + s0l;
      sv[1].status = STATES[s1h >> 4];
      sv[1].timeUs = ((s1h & 0xf) << 8) + s1l;
    }
};

ServoReader sr;

#include <ServoTimer1.h>
ServoTimer1 servo1, servo2;

#endif  // ENABLE_PWM

//---------------------------------------------------------------------


#ifdef ENABLE_GYRO

#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B);
#endif  // ENABLE_GYRO


//---------------------------------------------------------------------


#define BUFSIZE 200
class LineProcessor
{
    typedef void (*CallbackT)(const char* line);
    char buf[BUFSIZE];
    int cnt;
    bool waitForNewline;
    CallbackT callback;

  public:

    LineProcessor(CallbackT callback_)
      : cnt(0)
      , waitForNewline(true)
      , callback(callback_)
    {
      buf[cnt] = 0;
    }

    void addChar(uint8_t c) {
      if (cnt == BUFSIZE) {
        waitForNewline = true;
      }
      //CR and LF are control characters, respectively coded 0x0D (13 decimal) and 0x0A
      if (c == '\r' || c == '\n') {
        if ((!waitForNewline) && (cnt > 0)) {
          buf[cnt] = 0;
          callback(buf);
        }
        cnt = buf[0] = 0;
        waitForNewline = false;
      }
      else if (!waitForNewline) {
        buf[cnt] = c;
        ++cnt;
      }
    }
};


//---------------------------------------------------------------------


#ifdef ENABLE_GPS

#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
SoftwareSerial ss(RXPin, TXPin);

unsigned long lastGpggaTime = millis();
void gpsLineDone(const char* line)
{
  // http://aprs.gids.nl/nmea/#gga
  // Example: "4043.4476,N,07359.6348,W,1,6"
  //eg2. $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx
  //
  //hhmmss.ss = UTC of position
  //llll.ll = latitude of position
  //a = N or S
  //yyyyy.yy = Longitude of position
  //a = E or W
  //x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
  //xx = number of satellites in use
  //x.x = horizontal dilution of precision
  //x.x = Antenna altitude above mean-sea-level
  //M = units of antenna altitude, meters
  //x.x = Geoidal separation
  //M = units of geoidal separation, meters
  //x.x = Age of Differential GPS data (seconds)
  //xxxx = Differential reference station ID

  if (strncmp("$GPGGA,", line, 7)) {
    return;
  }
  // Don't forget to check the checksum on GPS logs.
  Serial.print("G ");
  Serial.print(line);
  Serial.print(" ");
  unsigned long now = millis();
  Serial.println(now - lastGpggaTime);
  lastGpggaTime = now;
}

LineProcessor np(gpsLineDone);

#endif  // ENABLE_GPS


// Anything more frequent than 14 updates/second will
// keep this under control. No data for more than 70 ms
// will revert to the remote control.
#define MILLISECOND_OVERRIDE 70

unsigned long lastSOverride = 0;
unsigned long lastTOverride = 0;
void consoleLineDone(const char* line)
{
  if (strncmp("S ", line, 2) == 0) {
    servo2.writeDirect(atoi(line + 2));
    lastSOverride = millis();
  }
  else if (strncmp("T ", line, 2) == 0) {
    servo1.writeDirect(atoi(line + 2));
    lastTOverride = millis();
  }
}

LineProcessor mp(consoleLineDone);


//---------------------------------------------------------------------

unsigned long lastGyroMeasure = micros();
unsigned long lastServoMeasure = micros();
unsigned long lastSonarMeasure = micros();

const int MPU = 0x68;

void setup() {

  // Init Serial
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }

  //---------------------------------------------------------------------

#ifdef ENABLE_GYRO2
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
#endif  // ENABLE_GYRO2

  //---------------------------------------------------------------------

#ifdef ENABLE_PWM
  sr.init();
  servo1.attach(9);
  servo2.attach(10);
#endif  // ENABLE_PWM

  //---------------------------------------------------------------------

#ifdef ENABLE_GYRO
  // Init Gyro
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(100);

  bno.setExtCrystalUse(true);
#endif  // ENABLE_GYRO


  //---------------------------------------------------------------------


  // GPS serial
#ifdef ENABLE_GPS
  ss.begin(57600);
#endif  // ENABLE_GPS
}

void loop()
{
#ifdef ENABLE_GPS
  while (ss.available() > 0) {
    np.addChar(ss.read());  // A Seria.print() might happen in here.
  }
#endif  // ENABLE_GPS

  while (Serial.available() > 0) {
    mp.addChar(Serial.read());
  }

  unsigned long now = micros();


  //---------------------------------------------------------------------


#if defined ENABLE_GYRO || defined ENABLE_GYRO2
  if ((now - lastGyroMeasure) > 500000) { // 2Hz

    Serial.print("A ");

#ifdef ENABLE_GYRO
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    sensors_event_t orientation, acceleration;
    bno.getEvent(&orientation);
    // Acceleration - gravity:
    bno.getEvent(&acceleration, Adafruit_BNO055::VECTOR_LINEARACCEL);

    Serial.print(orientation.orientation.x, 2);
    Serial.print(' ');
    Serial.print(orientation.orientation.y, 2);
    Serial.print(' ');
    Serial.print(orientation.orientation.z, 2);
    Serial.print(' ');
    Serial.print(acceleration.acceleration.x, 2);
    Serial.print(' ');
    Serial.print(acceleration.acceleration.y, 2);
    Serial.print(' ');
    Serial.print(acceleration.acceleration.z, 2);
#endif  // ENABLE_GYRO

#if defined ENABLE_GYRO && defined ENABLE_GYRO2
    Serial.print(' ');
#endif  // defined ENABLE_GYRO && defined ENABLE_GYRO2

#ifdef ENABLE_GYRO2
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
    {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true);
      AcX = Wire.read() << 8 | Wire.read();
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
      Wire.read();  // Tempeature H
      Wire.read();  // Tempeature L
      GyX = Wire.read() << 8 | Wire.read();
      GyY = Wire.read() << 8 | Wire.read();
      GyZ = Wire.read() << 8 | Wire.read();
    }

    Serial.print(AcX);
    Serial.print(' ');
    Serial.print(AcY);
    Serial.print(' ');
    Serial.print(AcZ);
    Serial.print(' ');
    Serial.print(GyX);
    Serial.print(' ');
    Serial.print(GyY);
    Serial.print(' ');
    Serial.print(GyZ);

#endif  // ENABLE_GYRO2

    Serial.println();

    lastGyroMeasure = now;
  }
#endif  // defined ENABLE_GYRO || defined ENABLE_GYRO2


  //---------------------------------------------------------------------


#ifdef ENABLE_SONAR
  if ((now - lastSonarMeasure ) > 500000) { // 2Hz
    int cm = sonar.ping_cm();
    Serial.print("R ");
    Serial.println(cm);
    lastSonarMeasure = now;
  }
#endif  // ENABLE_SONAR


  //---------------------------------------------------------------------

#ifdef ENABLE_PWM
  if ((now - lastServoMeasure) > 40000) { // 25Hz

    ServoValue sv[2];
    sr.read(sv);

    unsigned long nowMs = now / 1000;

    if (sv[0].status == 'D') {
      if (nowMs > (lastTOverride + MILLISECOND_OVERRIDE)) {
        servo1.writeDirect(sv[0].timeUs);
      }
      Serial.print("T ");
      Serial.println(sv[0].timeUs);
    }
    if (sv[1].status == 'D') {
      if (nowMs > (lastSOverride + MILLISECOND_OVERRIDE)) {
        servo2.writeDirect(sv[1].timeUs
                           // + int(100.0 * sin((now % 6000000)/1000000.))  // Drunk mode:
                          );
      }
      Serial.print("S ");
      Serial.println(sv[1].timeUs);
    }

    lastServoMeasure = now;
  }
#endif  // ENABLE_PWM
}
