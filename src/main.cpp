#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50  // 50Hz

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

void setServoAngle(uint8_t ch, int angle) {
  int pulse = angleToPulse(ch, angle);
  pwm.setPWM(ch, 0, pulse);
  Serial.printf("CH%d → %d° (pulse %d)\n", ch, angle, pulse);
}

void taskAutoMode(void *pvParameters) {
  enum State {
    RESET_ALL = 0,
    MOVE_CH0_45,
    RETURN_CH0_0,
    MOVE_CH2_45,
    RETURN_CH2_0,
    VERIFY_ALL_ZERO
  };

  State state = RESET_ALL;
  unsigned long stateStartTime = millis();

  while (1) {
    switch (state) {
      case RESET_ALL:
        setServoAngle(0, 0);
        setServoAngle(2, 0);
        Serial.println("[AUTO] Reset all to 0°");
        stateStartTime = millis();
        state = MOVE_CH0_45;
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // wait 5 sec
        break;

      case MOVE_CH0_45:
        setServoAngle(0, 45);
        Serial.println("[AUTO] CH0 → 45°");
        stateStartTime = millis();
        state = RETURN_CH0_0;
        vTaskDelay(10000 / portTICK_PERIOD_MS);  // wait 10 sec
        break;

      case RETURN_CH0_0:
        setServoAngle(0, 0);
        Serial.println("[AUTO] CH0 → 0°");
        stateStartTime = millis();
        state = MOVE_CH2_45;
        vTaskDelay(3000 / portTICK_PERIOD_MS);  // wait 3 sec
        break;

      case MOVE_CH2_45:
        setServoAngle(2, 45);
        Serial.println("[AUTO] CH2 → 45°");
        stateStartTime = millis();
        state = RETURN_CH2_0;
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // wait 5 sec
        break;

      case RETURN_CH2_0:
        setServoAngle(2, 0);
        Serial.println("[AUTO] CH2 → 0°");
        stateStartTime = millis();
        state = VERIFY_ALL_ZERO;
        vTaskDelay(3000 / portTICK_PERIOD_MS);  // wait 3 sec
        break;

      case VERIFY_ALL_ZERO:
        Serial.println("[AUTO] Verifying all servos at 0°...");
        state = RESET_ALL;
        break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // หน่วงเล็กน้อยระหว่าง state
  }
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  xTaskCreate(taskAutoMode, "AutoMode", 2048, NULL, 1, NULL);
}

void loop() {
  
}


