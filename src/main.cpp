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

// ช่อง (channel) บน PCA9685 สำหรับ LED ทั้งสองดวง
#define LED1_CH 0    // LED 1
#define LED2_CH 1    // LED 2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

  // ---------- เริ่มต้น PCA9685 สำหรับควบคุม LED ----------
  pwm.begin();
  pwm.setPWMFreq(1000);

  pwm.setPWM(LED1_CH, 0, 0);
  pwm.setPWM(LED2_CH, 0, 0);

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

  for (;;) {
    int dir1 = mapDirection(ibusData.ch[0]);
    int dir2 = mapDirection(ibusData.ch[1]);
    int dir3 = mapDirection(ibusData.ch[2]);

    if (dir3 == 1) {
      pwm.setPWM(LED1_CH, 0, 4095);
      pwm.setPWM(LED2_CH, 0, 4095);
    } else if (dir3 == -1) {
      pwm.setPWM(LED1_CH, 0, 0);
      pwm.setPWM(LED2_CH, 0, 0);
    } else {
      if (dir1 == 1) {
        pwm.setPWM(LED1_CH, 0, 4095);
      } else {
        pwm.setPWM(LED1_CH, 0, 0);
      }

      if (dir2 == 1) {
        pwm.setPWM(LED2_CH, 0, 4095);
      } else {
        pwm.setPWM(LED2_CH, 0, 0);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

