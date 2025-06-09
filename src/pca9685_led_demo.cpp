#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pwm.begin();
  pwm.setPWMFreq(1000); // Set frequency for LED
}

void loop() {
  const uint16_t duty = 2048; // 50% of 4096
  for (uint8_t ch = 0; ch < 4; ch++) {
    pwm.setPWM(ch, 0, duty); // Turn channel on at 50%
    delay(200);
    pwm.setPWM(ch, 0, 0);    // Turn channel off
  }
}

