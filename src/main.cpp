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

uint8_t BoosterAddress[] = { 0x9c, 0x13, 0x9e, 0x55, 0x1b, 0xa8 }; // ESP32 RGB错接到usb_vbus版本

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

volatile bool esp_now_connected, recvSucceed;
unsigned long lastSendTime = 0;
#define CONNECTION_TIMEOUT 500

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
#define STANDARD_BRIGHTNESS 10
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

/*----------------------------------------------- RGB LED-----------------------------------------------*/

#define RGB_LED_PIN 12
#define LONG_BLINK_DURATION 1000
#define SHORT_BLINK_DURATION 200
#define LONG_BLINK_INTERVAL 300
#define SHORT_BLINK_INTERVAL 100

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

/**  指示灯
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
    myRGB.clear();
    myRGB.show();
    vTaskDelay(interval / portTICK_PERIOD_MS);
  }
}
// 多色闪烁
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
    myRGB.clear();
    myRGB.show();
    vTaskDelay(interval / portTICK_PERIOD_MS);
  }
}

// ESP NOW
void esp_now_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int colors[] = { red, green, blue };
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    // 注册通信频道
    memcpy(peerInfo.peer_addr, BoosterAddress, 6);
    peerInfo.channel = 1;
    // 主初始化错误检查
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      buzzer(3, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
      esp_now_connected = false;
      myRGB.clear();
      myRGB.setPixelColor(0, red);
      myRGB.show();
#if DEBUG
      Serial.println("添加peer失败");
#endif
    } else {
      // 测试连接
      if (esp_now_send(BoosterAddress, (uint8_t*)&footPad, sizeof(footPad)) == ESP_OK) {
        esp_now_connected = true;
        // 成功指示灯提示
        mutipleColorBlink(colors, 3, LONG_FLASH_DURATION, LONG_FLASH_INTERVAL);
        buzzer(1, LONG_BEEP_DURATION, LONG_BEEP_INTERVAL);
#if DEBUG
        Serial.println("ESP NOW 初始化成功");
#endif
        return; // 成功就直接返回，不需要重连逻辑，直接跳出整个函数
      } else {
        esp_now_connected = false;
        myRGB.clear();
        myRGB.setPixelColor(0, red);
        myRGB.show();
        buzzer(3, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
      }
    }
  } else {
    esp_now_connected = false;
  }
  if (!esp_now_connected) {
#if DEBUG
    Serial.println("ESP NOW 初始化失败，正在重连...");
#endif
    // 尝试重连3次
    bool reconnectSuccess = false;
    for (int i = 0; i < 3; i++) {
#if DEBUG
      Serial.printf("重连第 %d 次...\n", i + 1);
#endif
      esp_now_init();
      esp_now_register_send_cb(OnDataSent);
      esp_now_register_recv_cb(OnDataRecv);
      memcpy(peerInfo.peer_addr, BoosterAddress, 6);
      peerInfo.channel = 1;
      // 测试连接
      if (esp_now_send(BoosterAddress, (uint8_t*)&footPad, sizeof(footPad)) == ESP_OK) {
        reconnectSuccess  = true;
        esp_now_connected = true;
        // 成功指示灯提示
        mutipleColorBlink(colors, 3, LONG_FLASH_DURATION, LONG_FLASH_INTERVAL);
        buzzer(1, LONG_BEEP_DURATION, LONG_BEEP_INTERVAL);
#if DEBUG
        Serial.printf("重连第 %d 次成功\n", i + 1);
#endif
        return; // 跳出for循环（可以理解为找到要找到的答案了）
      } else {
        reconnectSuccess = false;
      }
    }
    if (!reconnectSuccess) {
      esp_now_connected = false;
      buzzer(5, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL); // 长时间错误提示
#if DEBUG
      Serial.println("ESP NOW 重连失败");
#endif
    }
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

/**  电量读取
 * @brief     读取电量，更新电压、电量两变量，并返回电量状态
 * @param     voltsPercentage: 电量百分比
 * @param     batvolts: 电压值
 * @return    BatteryState: 电量状态枚举
 */
BatteryState batteryReading() {
  battery.readMilliVolts(BATTERY_READING_AVERAGE);
  batvolts        = battery._voltage;
  voltsPercentage = battery._voltsPercentage;
  if (voltsPercentage >= 80) {
    return FULL;
  } else if (voltsPercentage >= 60) {
    return DECENT;
  } else if (voltsPercentage >= 40) {
    return MODERATE;
  } else if (voltsPercentage >= 20) {
    return DEPLETED;
  } else {
    return CRITICAL;
  }
}

/**  电量指示颜色判断
 * @brief     用ws2812b不同颜色指示电量状态
 * @param     batteryLED: 电量指示灯状态标志位
 */
void batteryFlash(BatteryState state) {
  if (batteryLED) {
#if DEBUG
    Serial.println("用户踩下按钮");
    Serial.println("/*****************************/");
    Serial.println("电池电量指示函数:");
    Serial.print("电池电压: ");
    Serial.print(batvolts);
    Serial.print("V, 电量百分比: ");
    Serial.print(voltsPercentage);
    Serial.println("%");
#endif
    switch (state) {
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
  } else {
    myRGB.clear();
  }
  myRGB.show();
}

/**  电量读取指示和报警任务
 * @brief     上电/按键唤醒显示电量30秒后自动休眠，低于设定阈值后循环报警
 * @param     R1: 分压电阻R1阻值
 * @param     R2: 分压电阻R2阻值
 * @param     flashOnMillis: 指示灯显示时间计时器
 * @param     lastWarningTime: 低电量报警计时器
 * @param     lastBlinkTime: 低电量指示灯闪烁计时器
 * @param     blinkState: 低电量指示灯闪烁状态标志位
 */
void batteryCheck(void* pvParameter) {
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
  unsigned long flashOnMillis   = 0;
  unsigned long lastWarningTime = 0;
  unsigned long lastBlinkTime   = 0;
  bool          blinkState      = false;
  while (1) {
    BatteryState state = batteryReading();
    if (state != CRITICAL) {
      if (flashOnMillis == 0) flashOnMillis = millis(); // 首次显示，记录时间
      if (millis() - flashOnMillis < BATTERY_LED_INTERVAL) {
        batteryFlash(state); // 定期进行电量检测，所以不直接把batteryReading函数作为形参传入
      } else {
        batteryLED    = false;
        flashOnMillis = 0; // 重置计时器
      }
    }
    // 低电量报警。小于等于20%视为低电量
    else {
      flashOnMillis     = 0;                                      // 重置正常显示计时器
      int blinkInterval = map(voltsPercentage, 0, 20, 300, 1500); // 根据电量百分比调整闪烁和报警频率。
      if (millis() - lastBlinkTime > blinkInterval) {
        blinkState    = !blinkState;
        lastBlinkTime = millis();
        if (blinkState) {
          myRGB.setPixelColor(0, red);
        } else {
          myRGB.clear();
        }
        myRGB.show();
      }
      // 非阻塞蜂鸣器报警
      if (millis() - lastWarningTime > BATTERY_WARNING_INTERVAL) {
        buzzer(3, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
        lastWarningTime = millis();
#if DEBUG
        Serial.print("!!! 低电量报警 !!! 电量: ");
        Serial.print(battery._voltsPercentage);
        Serial.println("%");
#endif
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// usb状态改变中断处理函数
void IRAM_ATTR handleUSBInterrupt() {
  usbPluggedIn = digitalRead(USB_PIN);
}

// 数据发送任务
void dataTransmit(void* pvParameter) {
  functionButton.setup(FUNCTION_PIN, INPUT_PULLUP);
  functionButton.attachLongPressStart(longPressed_callback);
  functionButton.attachClick(shortPressed_callback);
  functionButton.setPressMs(800);
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(12.5); // 频率 80Hz → 周期为 1/80 = 0.0125 秒 = 12.5 毫秒
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
  bool          lastConnectionState = false;
  bool          blinkState          = false;
  unsigned long lastBlinkTime       = 0;
  while (1) {
    unsigned long currentTime = millis();
    esp_now_connected         = (currentTime - lastSendTime <= CONNECTION_TIMEOUT); // 判断是否连接：检查上次发送时间是否超时
    // 状态变化时立即更新显示
    if (lastConnectionState != esp_now_connected) {
      lastConnectionState = esp_now_connected;
      if (esp_now_connected && !batteryLED) {  // 只有在保持连接且电池指示灯关闭时才显示连接状态
        // 连接恢复：蓝色常亮
        blinkState = false;
        myRGB.clear();
        myRGB.setPixelColor(0, blue);
        myRGB.show();
      } else if (!esp_now_connected && !batteryLED) {  // 只有在断开连接且电池指示灯关闭时才显示断开连接状态
        // 断开连接：立即显示红色
        blinkState    = true;
        lastBlinkTime = currentTime;
        myRGB.clear();
        myRGB.setPixelColor(0, red);
        myRGB.show();
        buzzer(3, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
      }
    }
    // 未连接时闪烁红色
    if (!esp_now_connected) {
      if (currentTime - lastBlinkTime > 300) {
        lastBlinkTime = currentTime;
        blinkState    = !blinkState;
        myRGB.clear();
        if (blinkState) myRGB.setPixelColor(0, red);
        myRGB.show();
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
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

  analogReadResolution(12);
  myRGB.begin();
  myRGB.setBrightness(STANDARD_BRIGHTNESS);

  esp_now_connect();
  footPad.stepData[3] = false; // 功能键初始状态为假

  attachInterrupt(USB_PIN, handleUSBInterrupt, CHANGE);

  xTaskCreate(dataTransmit, "dataTransmit", 1024 * 2, NULL, 1, NULL);
  xTaskCreate(batteryCheck, "batteryCheck", 1024 * 2, NULL, 1, NULL);
  xTaskCreate(esp_now_connection, "esp_now_connection", 1024 * 1, NULL, 1, NULL);

#if DEBUG
  Serial.println("脚控初始化完成");
#endif
}
void loop() {
}