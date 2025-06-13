#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IBusBM.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SERVO_FREQ 50  // 50Hz for servos

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
IBusBM ibus;

typedef struct {
  int16_t ch[8];
} IBusData_t;
static IBusData_t ibusData;

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

enum SystemState {
  STATE_MANUAL,
  STATE_AUTO_RESET_ALL,
  STATE_AUTO_MOVE_CH0_45,
  STATE_AUTO_RETURN_CH0_0,
  STATE_AUTO_MOVE_CH2_45,
  STATE_AUTO_RETURN_CH2_0,
  STATE_AUTO_VERIFY_ALL_ZERO
};
volatile SystemState systemState = STATE_MANUAL;
SystemState lastPrintState = STATE_MANUAL;

int lastServoAngle[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };

int angleToPulse(uint8_t channel, int angle) {
  int minPulse, maxPulse;
  switch (channel) {
    case 0: minPulse = 106; maxPulse = 550; break;
    case 2: minPulse = 100; maxPulse = 525; break;
    default: minPulse = 105; maxPulse = 515; break;
  }
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, minPulse, maxPulse);
}

void setServoAngle(uint8_t ch, int angle) {
  if (lastServoAngle[ch] != angle) {
    int pulse = angleToPulse(ch, angle);
    pwm.setPWM(ch, 0, pulse);
    Serial.printf("CH%u → %d° (pulse %d)\n", ch, angle, pulse);
    lastServoAngle[ch] = angle;
  }
}

int mapDirection(int val) {
  if (val < 1200) return -1;
  else if (val > 1800) return 1;
  else return 0;
}

void taskReadIBus(void* pvParameters) {
  for (;;) {
    taskENTER_CRITICAL(&myMutex);
    for (uint8_t i = 0; i < 8; i++) {
      ibusData.ch[i] = ibus.readChannel(i);
    }
    taskEXIT_CRITICAL(&myMutex);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void taskController(void* pvParameters) {
  uint32_t stateStart = millis();

  for (;;) {
    int ch1, ch2, ch6;
    taskENTER_CRITICAL(&myMutex);
    ch1 = ibusData.ch[0];
    ch2 = ibusData.ch[1];
    ch6 = ibusData.ch[5];
    taskEXIT_CRITICAL(&myMutex);

    if (ch6 > 1500 && systemState == STATE_MANUAL) {
      Serial.println("[STATE] → AUTO");
      systemState = STATE_AUTO_RESET_ALL;
      stateStart = millis();
    }

    switch (systemState) {
      case STATE_MANUAL: {
        if (lastPrintState != STATE_MANUAL) {
          Serial.println("\n===== STATE: MANUAL =====");
          lastPrintState = STATE_MANUAL;
        }
        int dir1 = mapDirection(ch1);
        int dir2 = mapDirection(ch2);
        static int lastDir1 = 99, lastDir2 = 99;
        if (dir1 != lastDir1 || dir2 != lastDir2) {
          Serial.printf("CH1: %d => dir1=%d | CH2: %d => dir2=%d\n", ch1, dir1, ch2, dir2);
          lastDir1 = dir1;
          lastDir2 = dir2;
        }
        setServoAngle(0, (dir2 == 1) ? 45 : 0);
        setServoAngle(1, (dir2 == -1) ? 45 : 0);
        setServoAngle(2, (dir1 == 1) ? 45 : 0);
        setServoAngle(3, (dir1 == -1) ? 45 : 0);
        break;
      }
      case STATE_AUTO_RESET_ALL:
        if (lastPrintState != STATE_AUTO_RESET_ALL) {
          Serial.println("\n===== AUTO: RESET_ALL =====");
          lastPrintState = STATE_AUTO_RESET_ALL;
        }
        setServoAngle(0, 0);
        setServoAngle(1, 0);
        setServoAngle(2, 0);
        setServoAngle(3, 0);
        if (millis() - stateStart >= 5000) {
          systemState = STATE_AUTO_MOVE_CH0_45;
          stateStart = millis();
        }
        break;
      case STATE_AUTO_MOVE_CH0_45:
        if (lastPrintState != STATE_AUTO_MOVE_CH0_45) {
          Serial.println("\n===== AUTO: CH0 → 45° =====");
          lastPrintState = STATE_AUTO_MOVE_CH0_45;
        }
        setServoAngle(0, 45);
        if (millis() - stateStart >= 5000) {
          systemState = STATE_AUTO_RETURN_CH0_0;
          stateStart = millis();
        }
        break;
      case STATE_AUTO_RETURN_CH0_0:
        if (lastPrintState != STATE_AUTO_RETURN_CH0_0) {
          Serial.println("\n===== AUTO: CH0 → 0° =====");
          lastPrintState = STATE_AUTO_RETURN_CH0_0;
        }
        setServoAngle(0, 0);
        if (millis() - stateStart >= 3000) {
          systemState = STATE_AUTO_MOVE_CH2_45;
          stateStart = millis();
        }
        break;
      case STATE_AUTO_MOVE_CH2_45:
        if (lastPrintState != STATE_AUTO_MOVE_CH2_45) {
          Serial.println("\n===== AUTO: CH2 → 45° =====");
          lastPrintState = STATE_AUTO_MOVE_CH2_45;
        }
        setServoAngle(2, 45);
        if (millis() - stateStart >= 6800) {
          systemState = STATE_AUTO_RETURN_CH2_0;
          stateStart = millis();
        }
        break;
      case STATE_AUTO_RETURN_CH2_0:
        if (lastPrintState != STATE_AUTO_RETURN_CH2_0) {
          Serial.println("\n===== AUTO: CH2 → 0° =====");
          lastPrintState = STATE_AUTO_RETURN_CH2_0;
        }
        setServoAngle(2, 0);
        if (millis() - stateStart >= 3000) {
          systemState = STATE_AUTO_VERIFY_ALL_ZERO;
          stateStart = millis();
        }
        break;
      case STATE_AUTO_VERIFY_ALL_ZERO:
        if (lastPrintState != STATE_AUTO_VERIFY_ALL_ZERO) {
          Serial.println("\n===== AUTO: VERIFY ALL ZERO =====");
          lastPrintState = STATE_AUTO_VERIFY_ALL_ZERO;
        }
        setServoAngle(0, 0);
        setServoAngle(1, 0);
        setServoAngle(2, 0);
        setServoAngle(3, 0);
        systemState = STATE_AUTO_RESET_ALL;
        stateStart = millis();
        break;
    }

    if (ch6 <= 1500 && systemState != STATE_MANUAL) {
      Serial.println("[STATE] → MANUAL");
      setServoAngle(0, 0);
      setServoAngle(1, 0);
      setServoAngle(2, 0);
      setServoAngle(3, 0);
      systemState = STATE_MANUAL;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  ibus.begin(Serial2, 1);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  xTaskCreate(taskReadIBus, "IBUS_task", 2048, NULL, 1, NULL);
  xTaskCreate(taskController, "Controller", 4096, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
