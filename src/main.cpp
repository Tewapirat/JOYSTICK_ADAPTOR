#include <Arduino.h>
#include <IBusBM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =============================
//  ประกาศตัวแปรและโครงสร้าง
// =============================

// ไลบรารีอ่านค่า IBus
IBusBM ibus;

// โครงสร้างสำหรับเก็บข้อมูลจาก IBus (สมมติว่ามี 8 แชนเนล)
typedef struct {
  int16_t ch[8];
} IBusData_t;

static IBusData_t ibusData;  // ตัวเก็บค่าช่องที่อ่านได้ล่าสุด

// ช่องสัญญาณ PCA9685 สำหรับ Servo
#define SERVO1_CHANNEL 0   // Servo 1
#define SERVO2_CHANNEL 1   // Servo 2
#define SERVO3_CHANNEL 2   // Servo 3
#define SERVO4_CHANNEL 3   // Servo 4

// ค่าพัลส์สำหรับมุมต่ำสุดและสูงสุดของ Servo
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_CENTER ((SERVO_MIN + SERVO_MAX) / 2)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// โหมดการทำงานของระบบ
enum Mode {
  MODE_MANUAL,
  MODE_AUTO_WAIT,
  MODE_AUTO
};
// create branch for current mode

static Mode currentMode = MODE_MANUAL;

// ฟังก์ชันช่วยแม็ปค่า PWM ให้เป็นทิศทาง (-1, 0, +1)
static int mapDirection(int val) {
  if (val < 1200)      return -1;
  else if (val > 1800) return  1;
  else                 return  0;
}

// =============================
//  ยืนยัน FreeRTOS Task โปรโตไทป์
// =============================
void taskReadIBus(void* pvParameters);
void taskControlServo(void* pvParameters);

// =============================
//  ฟังก์ชัน setup()
// =============================
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // ---------- เริ่มต้น Serial2 สำหรับ IBus ----------
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // เริ่มต้นไลบรารี IBus บน Serial2, packet length = 1
  ibus.begin(Serial2, 1);

  // ---------- เริ่มต้นไลบรารี PCA9685 สำหรับ Servo ----------
  pwm.begin();
  pwm.setPWMFreq(50); // กำหนดความถี่สำหรับควบคุม Servo

  // ตั้งค่า Servo ให้อยู่กึ่งกลางทั้งหมดเมื่อเริ่มต้น
  pwm.setPWM(SERVO1_CHANNEL, 0, SERVO_CENTER);
  pwm.setPWM(SERVO2_CHANNEL, 0, SERVO_CENTER);
  pwm.setPWM(SERVO3_CHANNEL, 0, SERVO_CENTER);
  pwm.setPWM(SERVO4_CHANNEL, 0, SERVO_CENTER);

  currentMode = MODE_MANUAL;

  // ---------- สร้าง FreeRTOS Task ----------
  xTaskCreatePinnedToCore(
    taskReadIBus,
    "IBUS_Task",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    taskControlServo,
    "SERVO_Task",
    3072,
    NULL,
    1,
    NULL,
    1
  );
}

// =============================
//  ฟังก์ชัน loop()
// =============================
void loop() {
  vTaskDelay(portMAX_DELAY);
}

// ====================================================
//  ฟังก์ชัน Task อ่านค่า IBus (taskReadIBus)
// ====================================================
void taskReadIBus(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    for (uint8_t i = 0; i < 8; i++) {
      ibusData.ch[i] = ibus.readChannel(i);
    }

    Serial.print("IBus Channels: ");
    for (uint8_t i = 0; i < 8; i++) {
      Serial.printf("CH%u=%d", i + 1, ibusData.ch[i]);
      if (i < 7) Serial.print(" | ");
    }
    Serial.println();

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ====================================================
//  ฟังก์ชัน Task ควบคุม Servo (taskControlServo)
// ====================================================
void taskControlServo(void* pvParameters) {
  (void)pvParameters;

  static uint8_t autoStep = 0;           // ขั้นตอนของ state machine ในโหมด AUTO
  static unsigned long autoTimer = 0;    // จับเวลาสำหรับ AUTO_WAIT และ AUTO

  for (;;) {
    int dir1 = mapDirection(ibusData.ch[0]);
    int dir2 = mapDirection(ibusData.ch[1]);
    int dir3 = mapDirection(ibusData.ch[5]);

    switch (currentMode) {
      case MODE_MANUAL:
        pwm.setPWM(SERVO1_CHANNEL, 0,
                   (dir1 == 1) ? SERVO_MAX : (dir1 == -1) ? SERVO_MIN : SERVO_CENTER);
        pwm.setPWM(SERVO2_CHANNEL, 0,
                   (dir2 == 1) ? SERVO_MAX : (dir2 == -1) ? SERVO_MIN : SERVO_CENTER);
        pwm.setPWM(SERVO3_CHANNEL, 0, SERVO_CENTER);
        pwm.setPWM(SERVO4_CHANNEL, 0, SERVO_CENTER);

        if (dir3 == 1) {
          currentMode = MODE_AUTO_WAIT;
          autoTimer = millis();
        }
        break;

      case MODE_AUTO_WAIT:
        pwm.setPWM(SERVO1_CHANNEL, 0, SERVO_CENTER);
        pwm.setPWM(SERVO2_CHANNEL, 0, SERVO_CENTER);
        pwm.setPWM(SERVO3_CHANNEL, 0, SERVO_CENTER);
        pwm.setPWM(SERVO4_CHANNEL, 0, SERVO_CENTER);

        if (dir3 == 1) {
          if (millis() - autoTimer >= 3000) {
            currentMode = MODE_AUTO;
            autoStep = 0;
            autoTimer = millis();
          }
        } else {
          currentMode = MODE_MANUAL;
        }
        break;

      case MODE_AUTO:
        if (dir3 != 1) {
          currentMode = MODE_MANUAL;
          break;
        }

        if (millis() - autoTimer >= 500) {
          autoTimer = millis();
          autoStep = (autoStep + 1) % 4;
        }

        pwm.setPWM(SERVO1_CHANNEL, 0, (autoStep == 0) ? SERVO_MAX : SERVO_CENTER);
        pwm.setPWM(SERVO2_CHANNEL, 0, (autoStep == 1) ? SERVO_MAX : SERVO_CENTER);
        pwm.setPWM(SERVO3_CHANNEL, 0, (autoStep == 2) ? SERVO_MAX : SERVO_CENTER);
        pwm.setPWM(SERVO4_CHANNEL, 0, (autoStep == 3) ? SERVO_MAX : SERVO_CENTER);
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

