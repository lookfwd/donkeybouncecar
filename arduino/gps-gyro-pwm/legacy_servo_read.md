# Old routine for reading PWM


```
#define SERVO_RESET 1500

class Channel {
  volatile bool lastState;
  volatile long lastLowToHigh;
  volatile int timeUs;
  volatile int lastValue;

public:
  Channel()
    : lastState(false)
    , lastLowToHigh(0)
    , timeUs(SERVO_RESET)
    , lastValue(SERVO_RESET)
  {
  }

  bool onChange(bool state)
  {
    if (state == lastState) {
      return false;
    }
    lastState = state;

    unsigned long now = micros();
    if (state) {
      // 0->1: Capture frame start
      lastLowToHigh = now;
      return;
    }

    // 1->0: Capture time
    timeUs = (now - lastLowToHigh);

    if (lastValue != timeUs) {
      lastValue = timeUs;
      return true;
    }
    return false;
  }

  int getTimeUs() {
    return timeUs;
  }
};

Channel d2throttle, d3steering;




uint8_t d2_mask, d3_mask;
volatile uint8_t *interruptPort;

void pinx_ISR() {
  // Way faster than digitalRead(). digitalRead() might cost 5us, which is bad
  // because it adds tons of jitter and we had to do it twice, once for each
  // pin.
  // https://forum.arduino.cc/index.php?topic=337578.msg2327508#msg2327508
  uint8_t portv = *interruptPort;
  if (d2throttle.onChange(portv & d2_mask)) {
    servo1.writeDirect(d2throttle.getTimeUs());
  }
  if (d3steering.onChange(portv & d3_mask)) {
    // Drunk mode:
    //  + int(100.0 * sin((micros() % 6000000)/1000000.))
    servo2.writeDirect(d3steering.getTimeUs());
  }
}


setup() {
  interruptPort = portInputRegister(digitalPinToPort(2));
  d2_mask = digitalPinToBitMask(2);
  d3_mask = digitalPinToBitMask(3);
  if (interruptPort != portInputRegister(digitalPinToPort(3)))
  {
    Serial.println("Ooops, Interrupt pins not in the same port");
    while(1);
  }

  // PWM Read
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), pinx_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), pinx_ISR, CHANGE);
}


```