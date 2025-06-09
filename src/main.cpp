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

// ช่องสัญญาณ PCA9685 สำหรับ LED ทั้งห้าดวง
#define LED1_CHANNEL 0    // LED 1
#define LED2_CHANNEL 1    // LED 2
#define LED3_CHANNEL 2    // LED 3
#define LED4_CHANNEL 3    // LED 4
#define LED5_CHANNEL 4    // LED 5

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define LED_ON  4095
#define LED_OFF 0

// โหมดการทำงานของระบบ
enum Mode {
  MODE_MANUAL,
  MODE_AUTO_WAIT,
  MODE_AUTO
};

static Mode currentMode = MODE_MANUAL;
static bool led5Blink = false;  // ควบคุมการกระพริบของ LED5

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
void taskControlLED(void* pvParameters);
void taskFlashLED5(void* pvParameters);

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

  // ---------- เริ่มต้นไลบรารี PCA9685 สำหรับ LED ----------
  pwm.begin();
  pwm.setPWMFreq(1000); // กำหนดความถี่สำหรับควบคุม LED

  // ตั้งค่า LED ทั้งหมดดับก่อนและเปิด LED5 ค้างสำหรับโหมด manual
  pwm.setPWM(LED1_CHANNEL, 0, LED_OFF);
  pwm.setPWM(LED2_CHANNEL, 0, LED_OFF);
  pwm.setPWM(LED3_CHANNEL, 0, LED_OFF);
  pwm.setPWM(LED4_CHANNEL, 0, LED_OFF);
  pwm.setPWM(LED5_CHANNEL, 0, LED_ON);

  currentMode = MODE_MANUAL;
  led5Blink = false;

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
    taskControlLED,
    "LED_Task",
    3072,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    taskFlashLED5,
    "FLASH_Task",
    2048,
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
//  ฟังก์ชัน Task ควบคุม LED (taskControlLED)
// ====================================================
void taskControlLED(void* pvParameters) {
  (void)pvParameters;

  static uint8_t autoStep = 0;           // ขั้นตอนของ state machine ในโหมด AUTO
  static unsigned long autoTimer = 0;    // จับเวลาสำหรับ AUTO_WAIT และ AUTO

  for (;;) {
    int dir1 = mapDirection(ibusData.ch[0]);
    int dir2 = mapDirection(ibusData.ch[1]);
    int dir3 = mapDirection(ibusData.ch[2]);

    switch (currentMode) {
      case MODE_MANUAL:
        led5Blink = false;
        pwm.setPWM(LED5_CHANNEL, 0, LED_ON);

        pwm.setPWM(LED1_CHANNEL, 0, (dir1 == 1) ? LED_ON : LED_OFF);
        pwm.setPWM(LED2_CHANNEL, 0, (dir1 == -1) ? LED_ON : LED_OFF);
        pwm.setPWM(LED3_CHANNEL, 0, (dir2 == 1) ? LED_ON : LED_OFF);
        pwm.setPWM(LED4_CHANNEL, 0, (dir2 == -1) ? LED_ON : LED_OFF);

        if (dir3 == 1) {
          currentMode = MODE_AUTO_WAIT;
          autoTimer = millis();
        }
        break;

      case MODE_AUTO_WAIT:
        led5Blink = true;
        pwm.setPWM(LED1_CHANNEL, 0, LED_OFF);
        pwm.setPWM(LED2_CHANNEL, 0, LED_OFF);
        pwm.setPWM(LED3_CHANNEL, 0, LED_OFF);
        pwm.setPWM(LED4_CHANNEL, 0, LED_OFF);

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
        led5Blink = true;

        if (dir3 != 1) {
          currentMode = MODE_MANUAL;
          break;
        }

        if (millis() - autoTimer >= 500) {
          autoTimer = millis();
          autoStep = (autoStep + 1) % 4;
        }

        pwm.setPWM(LED1_CHANNEL, 0, (autoStep == 0) ? LED_ON : LED_OFF);
        pwm.setPWM(LED2_CHANNEL, 0, (autoStep == 1) ? LED_ON : LED_OFF);
        pwm.setPWM(LED3_CHANNEL, 0, (autoStep == 2) ? LED_ON : LED_OFF);
        pwm.setPWM(LED4_CHANNEL, 0, (autoStep == 3) ? LED_ON : LED_OFF);
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ====================================================
//  ฟังก์ชัน Task กระพริบ LED5 (taskFlashLED5)
// ====================================================
void taskFlashLED5(void* pvParameters) {
  (void)pvParameters;

  bool state = false;
  unsigned long lastToggle = 0;

  for (;;) {
    if (led5Blink) {
      if (millis() - lastToggle >= 1000) {
        lastToggle = millis();
        state = !state;
        pwm.setPWM(LED5_CHANNEL, 0, state ? LED_ON : LED_OFF);
      }
    } else {
      if (!state) {
        state = true;
        pwm.setPWM(LED5_CHANNEL, 0, LED_ON);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

