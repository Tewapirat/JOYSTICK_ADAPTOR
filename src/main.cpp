#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50  // 50Hz สำหรับเซอร์โว

int angleToPulse(int channel, int angle) {
  int minPulse, maxPulse;

  switch (channel) {
    case 0:
      minPulse = 106;
      maxPulse = 550;
      break;
    case 2:
      minPulse = 100;
      maxPulse = 525;
      break;
    default:
      minPulse = 105;
      maxPulse = 515;
      break;
  }

  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, minPulse, maxPulse);
}

void taskServo(void *pvParameters) {
  const uint8_t channels[] = {0, 2};
  const int angles[] = {0, 90, 180};
  const int delayMs = 1000;

  while (1) {
    // หมุนทีละเซอร์โว → หมุนทุกมุม
    for (int j = 0; j < sizeof(channels) / sizeof(channels[0]); j++) {
      int ch = channels[j];
      for (int i = 0; i < sizeof(angles) / sizeof(angles[0]); i++) {
        int angle = angles[i];
        int pulse = angleToPulse(ch, angle);
        pwm.setPWM(ch, 0, pulse);
        Serial.printf("CH%d → %d° = pulse %d\n", ch, angle, pulse);
        vTaskDelay(delayMs / portTICK_PERIOD_MS);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // crystal 27 MHz
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  xTaskCreate(taskServo, "ServoOneByOne", 2048, NULL, 1, NULL);
}

void loop() {
  // ไม่ใช้ loop()
}
