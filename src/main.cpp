#include "OneButton.h"
#include "Wire.h"
#include "batteryReading.hpp"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define DEBUG 0

/*----------------------------------------------- ESP NOW-----------------------------------------------*/

// uint8_t BoosterAddress[] = { 0x9c, 0x13, 0x9e, 0x55, 0x1b, 0xa8 }; // ESP32 RGB错接到usb_vbus版本
uint8_t BoosterAddress[] = { 0xb4, 0x3a, 0x45, 0x46, 0x87, 0xd0 }; // 电推ver2.0版本MAC地址

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

// 声明一个枚举类型的变量，与接收端对应
enum BoosterState {
  HAND_MODE,
  FOOT_MODE,
  CRUISE_MODE,
  STANDBY_MODE
};

// 声明一个结构体，用来接收数据
struct Booster {
  BoosterState mode;
};
Booster booster;

// 发送的数据
struct FootPad {
  bool stepData[4] = {}; // 0、左转，1、右转，2、电推，3、功能
  int  stepSpeed;        // 步进电机转速
};
FootPad footPad;

volatile bool esp_now_connected, sendSucceed, recvSucceed;
unsigned long lastSendTime = 0; // 上次发送时间
#define CONNECTION_TIMEOUT 500  // 连接超时时间，单位毫秒
#define BLINK_INTERVAL 500      // 断线指示灯闪烁间隔

/*----------------------------------------- MPU6050 & QMC5883P -----------------------------------------*/

#define SDA_PIN 21
#define SCL_PIN 22

// MPU6050 mpu;
// float yaw, gyro_Z; // Z轴角度，Z轴角速度
// MPU6050 mpu6050(Wire);

/*----------------------------------------------- 操控 -----------------------------------------------*/

#define STEP_TURN_LEFT 32
#define STEP_TURN_RIGHT 19
#define THROTTLE_PIN 25
#define FUNCTION_PIN 33
#define STEP_SPEED 36

OneButton functionButton;

/*---------------------------------------------- 蜂鸣器 ----------------------------------------------*/

#define BUZZER_PIN 18
#define LONG_BEEP_DURATION 1000 // 长鸣时间
#define SHORT_BEEP_DURATION 200 // 短鸣时间
#define LONG_BEEP_INTERVAL 300  // 长鸣间隔时间
#define SHORT_BEEP_INTERVAL 100 // 短鸣间隔时间

/*----------------------------------------------- WS2812 -----------------------------------------------*/

#define WS2812_PIN 17
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0
#define STANDARD_BRIGHTNESS 100
#define SHORT_FLASH_DURATION 200
#define SHORT_FLASH_INTERVAL 200
#define LONG_FLASH_DURATION 500
#define LONG_FLASH_INTERVAL 500

Adafruit_NeoPixel myRGB(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
int               red    = myRGB.Color(255, 0, 0);   // 红色
int               green  = myRGB.Color(0, 255, 0);   // 绿色
int               blue   = myRGB.Color(0, 0, 255);   // 蓝色
int               cyan   = myRGB.Color(0, 180, 255); // 青色
int               yellow = myRGB.Color(255, 40, 0);  // 黄色

unsigned long lastBlinkTime = 0;     // 上次闪烁时间记录
bool          ledState      = false; // LED状态标志位

/*----------------------------------------------- RGB LED-----------------------------------------------*/

#define RGB_LED_PIN 12

/*----------------------------------------------- 电池电量 -----------------------------------------------*/

#define BATTERY_PIN 39
#define USB_PIN 34
#define BATTERY_MAX_VALUE 4.2                  // 电池最大电量
#define BATTERY_MIN_VALUE 3.2                  // 电池最小电量
#define BATTERY_READING_AVERAGE 50             // 均值滤波采样次数
#define BATTERY_READING_INTERVAL 3 * 60 * 1000 // 采样间隔 3分钟*60秒*1000毫秒
#define BATTERY_WARNING_INTERVAL 1 * 60 * 1000 // 低电量报警间隔
#define BATTERY_LED_INTERVAL 30000             // 指示灯休眠间隔

#define R1 9990
#define R2 9980

enum BatteryState {
  FULL,     // 100% - 80%
  DECENT,   // 80%  - 60%
  MODERATE, // 60%  - 40%
  DEPLETED, // 40%  - 20%
  CRITICAL  // 20%  - 0%
};
BatteryState batteryState;

float         batvolts, voltsPercentage;
volatile bool batteryLED   = true;
volatile bool usbPluggedIn = false;

BatReading battery;

/*----------------------------------------------- 自定义函数 -----------------------------------------------*/

// 数据发出去之后的回调函数
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // 如果发送成功
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSendTime = millis();
    if (!sendSucceed) sendSucceed = true;
  } else {
    sendSucceed = false;
  }
}

// 收到消息后的回调
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&booster, incomingData, sizeof(booster));
  if (!recvSucceed) recvSucceed = true;
}

/**  蜂鸣器
 * @brief     蜂鸣器通用函数
 * @param     times: 鸣叫次数
 * @param     duration: 持续时间，单位毫秒
 * @param     reverse: 每次鸣叫的间隔时间，单位毫秒
 */
void buzzer(uint8_t times, int duration, int interval) {
  if (times == 1) interval = 0; // 如果只鸣叫一次则不间隔
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(interval / portTICK_PERIOD_MS);
  }
}

/**  通用单色闪烁函数
 * @brief     适用于单个颜色闪烁
 * @param     times:    闪烁次数
 * @param     duration: 持续时间，单位毫秒
 * @param     interval: 每次闪烁的间隔时间，单位毫秒
 * @param     color:    颜色值
 */
void rgbBlink(int rgb_num, int times, int duration, int interval, int color) {
  if (times == 1) interval = 0; // 如果只闪烁一次则不间隔
  for (int i = 0; i < times; i++) {
    myRGB.clear();
    myRGB.setPixelColor(0, color); // led编号和颜色，编号从0开始。
    myRGB.show();
    vTaskDelay(duration / portTICK_PERIOD_MS);
    myRGB.setPixelColor(0, 0);
    myRGB.show();
    vTaskDelay(interval / portTICK_PERIOD_MS);
  }
}

/**  通用单色无限闪烁函数
 * @brief     适用于单个颜色闪烁
 * @param     interval: 每次闪烁的间隔时间，单位毫秒
 * @param     color:    颜色值
 */
void rgbInfiniteBlink(int interval, int color) {
  unsigned long currentTime = millis();
  if (currentTime - lastBlinkTime > interval) {
    lastBlinkTime = currentTime;
    ledState      = !ledState;
    myRGB.setPixelColor(0, ledState ? color : 0);
    myRGB.show();
  }
}

/**  通用多色闪烁函数
 * @brief     适用于单个颜色闪烁
 * @param     colors:    颜色数组
 * @param     colorNum: 颜色数量
 */
void mutipleColorBlink(int colors[], int colorNum, int duration, int interval) {
  for (int i = 0; i < colorNum; i++) {
    myRGB.clear();
    myRGB.setPixelColor(0, colors[i]);
    myRGB.show();
    vTaskDelay(duration / portTICK_PERIOD_MS);
    myRGB.setPixelColor(0, 0);
    myRGB.show();
    vTaskDelay(interval / portTICK_PERIOD_MS);
  }
}

/**
 * @brief WS2812呼吸灯效果
 * @param color 颜色值
 * @param cycles 呼吸循环次数（0为无限）
 * @param duration 单次呼吸总时间（毫秒）
 * @param minBrightness 最小亮度（0-255）
 * @param maxBrightness 最大亮度（0-255）
 */
void rgbBreath(int color, int cycles = 1, int duration = 2000, int minBrightness = 10, int maxBrightness = 100) {
  int  stepTime = duration / (2 * (maxBrightness - minBrightness));
  bool infinite = (cycles == 0);
  int  count    = 0;
  while (infinite || count < cycles) {
    // 渐亮
    for (int i = minBrightness; i <= maxBrightness; i++) {
      myRGB.clear();
      myRGB.setBrightness(i);
      myRGB.setPixelColor(0, color);
      myRGB.show();
      vTaskDelay(stepTime / portTICK_PERIOD_MS);
    }
    // 渐暗
    for (int i = maxBrightness; i >= minBrightness; i--) {
      myRGB.clear();
      myRGB.setBrightness(i);
      myRGB.setPixelColor(0, color);
      myRGB.show();
      vTaskDelay(stepTime / portTICK_PERIOD_MS);
    }
    count++;
  }
  // 恢复标准亮度
  myRGB.setBrightness(STANDARD_BRIGHTNESS);
}

// ESP NOW 初始化
void esp_now_connect() {
  int colors[] = { red, green, blue };
  int colorNum = 3;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW 初始化失败");
    esp_now_connected = false;
    myRGB.clear();
    myRGB.setPixelColor(0, red);
    myRGB.show();
    return;
  } else {
    Serial.println("ESP NOW 初始化成功");
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  memcpy(peerInfo.peer_addr, BoosterAddress, 6);
  peerInfo.channel = 1;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP NOW 添加对等节点失败");
    esp_now_connected = false;
    myRGB.clear();
    myRGB.setPixelColor(0, red);
    myRGB.show();
    return;
  } else {
    Serial.println("ESP NOW 添加对等节点成功");
  }
  esp_now_send(BoosterAddress, (uint8_t*)&booster, sizeof(booster));
  if (sendSucceed && recvSucceed) {
    Serial.println("ESP NOW 初始化，发送测试数据并接收成功");
    esp_now_connected = true;
    myRGB.clear();
    myRGB.setPixelColor(0, blue);
    myRGB.show();
    mutipleColorBlink(colors, colorNum, LONG_FLASH_DURATION, LONG_FLASH_INTERVAL);
  } else {
    Serial.println("ESP NOW 发送失败，正在重试...");
    for (int i = 0; i < 60; i++) {
      // 状态指示：红灯闪烁表示正在尝试连接
      myRGB.setPixelColor(0, 0);
      myRGB.setPixelColor(0, red);
      myRGB.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);
      myRGB.setPixelColor(0, 0);
      myRGB.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);
      esp_now_send(BoosterAddress, (uint8_t*)&booster, sizeof(booster));
      if (!sendSucceed || !recvSucceed) {
        Serial.print(".");
        continue;
      }
      // 成功处理
      myRGB.clear();
      myRGB.setPixelColor(0, blue);
      myRGB.show();
      esp_now_connected = true;
      mutipleColorBlink(colors, colorNum, LONG_FLASH_DURATION, LONG_FLASH_INTERVAL);
      Serial.printf("第%d次重试成功\n", i + 1);
      return;
    }
    Serial.println("ESP NOW 发送失败，重试次数已达上限");
    esp_now_connected = false;
    sendSucceed       = false;
    recvSucceed       = false;
    myRGB.clear();
    myRGB.setPixelColor(0, red);
    myRGB.show();
  }
}

// 功能按键长按回调函数
void longPressed_callback() {
  footPad.stepData[3] = !footPad.stepData[3];
  buzzer(1, LONG_BEEP_DURATION, LONG_BEEP_INTERVAL);
#if DEBUG
  if (footPad.stepData[3]) {
    Serial.println("功能键长按");
  }
#endif
}

// 功能按键短按回调函数
void shortPressed_callback() {
  batteryLED = true;
  buzzer(1, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
#if DEBUG
  if (footPad.stepData[3]) {
    Serial.println("功能键短按");
  }
#endif
}

// usb状态改变中断处理函数
void IRAM_ATTR handleUSBInterrupt() {
  usbPluggedIn = digitalRead(USB_PIN);
}

// WS2812系统状态及电量指示任务
void ws2812b_task(void* pvParameters) {
  unsigned long lastBatteryLEDTime  = 0;
  bool          lastConnectionState = false;
  while (1) {
    // 电量指示
    if (batteryLED) {
      unsigned long currentTime = millis();
      if (currentTime - lastBatteryLEDTime > BATTERY_LED_INTERVAL) {
        lastBatteryLEDTime = currentTime;
        myRGB.clear();
        // 根据电量状态设置颜色
        switch (batteryState) {
        case FULL:
          myRGB.setPixelColor(0, cyan);
          break;
        case DECENT:
          myRGB.setPixelColor(0, green);
          break;
        case MODERATE:
          myRGB.setPixelColor(0, yellow);
          break;
        case DEPLETED:
          myRGB.setPixelColor(0, red);
          break;
        default:
          break;
        }
        myRGB.show();
        batteryLED = false;
      }
    } else {
      if (lastConnectionState != esp_now_connected) {
        lastConnectionState = esp_now_connected;
        if (esp_now_connected) {
          rgbInfiniteBlink(LONG_FLASH_INTERVAL, blue);
        } else {
          myRGB.clear();
          myRGB.setPixelColor(0, red);
          myRGB.show();
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**  电量读取指示和报警任务
 * @brief     上电/按键唤醒显示电量30秒后自动休眠，低于设定阈值后循环报警
 * @param     R1: 分压电阻R1阻值
 * @param     R2: 分压电阻R2阻值
 * @param     voltsPercentage: 电量百分比
 * @param     batvolts: 电压值
 */
void batteryCheck(void* pvParameter) {
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
  while (1) {
    battery.readMilliVolts(BATTERY_READING_AVERAGE);
    batvolts        = battery._voltage;
    voltsPercentage = battery._voltsPercentage;
    if (voltsPercentage >= 80) {
      batteryState = FULL;
    } else if (voltsPercentage >= 60) {
      batteryState = DECENT;
    } else if (voltsPercentage >= 40) {
      batteryState = MODERATE;
    } else if (voltsPercentage >= 20) {
      batteryState = DEPLETED;
    } else {
      batteryState = CRITICAL;
      buzzer(3, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
    }
    vTaskDelay(voltsPercentage * BATTERY_READING_INTERVAL / portTICK_PERIOD_MS); // 电量越低读取越频繁
  }
}

// 数据发送任务
void dataTransmit(void* pvParameter) {
  functionButton.setup(FUNCTION_PIN, INPUT_PULLUP);
  functionButton.attachLongPressStart(longPressed_callback);
  functionButton.attachClick(shortPressed_callback);
  functionButton.setPressMs(800);
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(20); // 单位ms，数据发送频率为50Hz。换算为频率： 50Hz → 周期为 1000ms/20ms = 50 次/秒
  while (1) {
    functionButton.tick();
    footPad.stepData[0] = digitalRead(STEP_TURN_LEFT);  // 左转
    footPad.stepData[1] = digitalRead(STEP_TURN_RIGHT); // 右转
    footPad.stepData[2] = digitalRead(THROTTLE_PIN);    // 电推油门
    footPad.stepSpeed   = analogRead(STEP_SPEED);
#if DEBUG
    if (footPad.stepData[0] == LOW && footPad.stepData[1] == HIGH) {
      Serial.println("左转");
      Serial.printf("步进电机转速: %d\n", footPad.stepSpeed);
    } else if (footPad.stepData[0] == HIGH && footPad.stepData[1] == LOW) {
      Serial.println("右转");
      Serial.printf("步进电机转速: %d\n", footPad.stepSpeed);
    }
    if (footPad.stepData[2] == LOW) {
      Serial.println("踩油门");
    }
#endif
    esp_now_send(BoosterAddress, (uint8_t*)&footPad, sizeof(footPad));
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ESP NOW 连接检测任务
void esp_now_connection(void* pvParameter) {
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(1000); // 单位ms，数据检测频率为1Hz。换算为频率： 1Hz → 周期为 1000ms/1000ms = 1 次/秒
  while (1) {
    unsigned long currentTime = millis();
    esp_now_connected         = (currentTime - lastSendTime <= CONNECTION_TIMEOUT);
  }
  vTaskDelayUntil(&xLastWakeTime, xPeriod);
}

/*-------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RGB_LED_PIN, OUTPUT);

  pinMode(STEP_TURN_LEFT, INPUT_PULLUP);
  pinMode(STEP_TURN_RIGHT, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(FUNCTION_PIN, INPUT_PULLUP);
  pinMode(USB_PIN, INPUT_PULLDOWN);

  digitalWrite(RGB_LED_PIN, HIGH); // 关闭板载RGB LED
  analogReadResolution(12);
  myRGB.begin();
  myRGB.setBrightness(STANDARD_BRIGHTNESS);

  esp_now_connect();
  footPad.stepData[3] = false; // 功能键初始状态为假
  batteryLED          = true;  // 上电时显示电量指示灯

  attachInterrupt(USB_PIN, handleUSBInterrupt, CHANGE);

  xTaskCreate(dataTransmit, "dataTransmit", 1024 * 2, NULL, 2, NULL);
  xTaskCreate(batteryCheck, "batteryCheck", 1024 * 2, NULL, 1, NULL);
  xTaskCreate(esp_now_connection, "esp_now_connection", 1024 * 2, NULL, 1, NULL);
  xTaskCreate(ws2812b_task, "ws2812b_task", 1024 * 2, NULL, 1, NULL);

#if DEBUG
  Serial.println("脚控初始化完成");
#endif
}
void loop() {
}