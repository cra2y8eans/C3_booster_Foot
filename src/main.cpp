#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
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

uint8_t BoosterAddress[] = { 0x9c, 0x13, 0x9e, 0x55, 0x1b, 0xa8 }; // 测试板
// uint8_t BoosterAddress[] = { 0x24, 0x58, 0x7c, 0x91, 0xdf, 0x44 }; // super mini 排针

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

MPU6050 mpu;
// float yaw, gyro_Z; // Z轴角度，Z轴角速度
// MPU6050 mpu6050(Wire);

/*----------------------------------------------- 操控 -----------------------------------------------*/

#define STEP_TURN_LEFT 32
#define STEP_TURN_RIGHT 19
#define THROTTLE_PIN 25
#define FUNCTION_PIN 33
#define STEP_SPEED 36

OneButton functionButton;
// OneButton batteryButton;

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
#define FLASH_INTERVAL 30000

uint16_t brightness; // 动态亮度

Adafruit_NeoPixel myRGB(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
uint32_t          red    = myRGB.Color(255, 0, 0);  // 红色
uint32_t          green  = myRGB.Color(0, 255, 0);  // 绿色
uint32_t          blue   = myRGB.Color(0, 0, 255);  // 蓝色
uint32_t          yellow = myRGB.Color(255, 40, 0); // 黄色

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
#define BATTERY_MIN_PERCENTAGE 20              // 电池最低百分比
#define BATTERY_READING_AVERAGE 50             // 采样次数
#define BATTERY_READING_INTERVAL 3 * 60 * 1000 // 采样间隔 3分钟*60秒*1000毫秒
#define R1 9990
#define R2 9980

enum BatteryState {
  SEVENTY_PLUS,
  FIFTY_TO_SEVENTY,
  THIRTY_TO_FIFTY,
  BELOW_THIRTY
};
BatteryState batteryState;

float         batvolts, voltsPercentage;
volatile bool batteryLED   = true;
volatile bool batteryLow   = false;
volatile bool usbConnected = false;

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

// ESP NOW
void esp_now_connect() {
  WiFi.mode(WIFI_STA); // 设置wifi为STA模式
  WiFi.begin();
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
    esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
    // 注册通信频道
    memcpy(peerInfo.peer_addr, BoosterAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
    peerInfo.channel = 1;                          // 设置通信频道
    esp_now_add_peer(&peerInfo);                   // 添加通信对象
    // 指示灯提示
    myRGB.clear();
    myRGB.setPixelColor(0, red);
    myRGB.show();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    myRGB.clear();
    myRGB.setPixelColor(0, green);
    myRGB.show();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    myRGB.clear();
    myRGB.setPixelColor(0, blue);
    myRGB.show();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    myRGB.clear();
#if DEBUG
    Serial.println("ESP NOW 初始化成功");
#endif
  } else {
// 如果初始化失败则重连
#if DEBUG
    Serial.println("ESP NOW 初始化失败，正在重连...");
#endif
    // 报警
    buzzer(3, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
    // 尝试重连3次
    bool reconnect_3_times = false;
    while (!reconnect_3_times) {
      for (int i = 0; i < 3; i++) {
        buzzer(1, LONG_BEEP_DURATION, LONG_BEEP_INTERVAL);
// 重连
#if DEBUG
        Serial.printf("重连第 %d 次...\n", i + 1);
#endif
        esp_now_init();                                // 初始化ESP NOW
        esp_now_register_send_cb(OnDataSent);          // 注册发送成功的回调函数
        esp_now_register_recv_cb(OnDataRecv);          // 注册接受数据后的回调函数
        memcpy(peerInfo.peer_addr, BoosterAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
        peerInfo.channel = 1;                          // 设置通信频道
        esp_now_add_peer(&peerInfo);                   // 添加通信对象
        vTaskDelay(5000 / portTICK_PERIOD_MS);         // 延时5秒
      }
      reconnect_3_times = true; // 如果3次重连都失败，则退出循环
      esp_now_connected = false;
      digitalWrite(RGB_LED_PIN, HIGH);
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
 * @param     batteryLow: 低电量标志位
 * @param     voltsPercentage: 电量百分比
 * @param     batvolts: 电压值
 * @return    BatteryState: 电量状态枚举
 */
BatteryState batteryReading() {
  battery.readMilliVolts(BATTERY_READING_AVERAGE);
  batvolts        = battery._voltage;
  voltsPercentage = battery._voltsPercentage;
  if (voltsPercentage >= 70) {
    batteryLow = false;
    return SEVENTY_PLUS;
  } else if (voltsPercentage >= 50) {
    batteryLow = false;
    return FIFTY_TO_SEVENTY;
  } else if (voltsPercentage >= 30) {
    batteryLow = false;
    return THIRTY_TO_FIFTY;
  } else {
    batteryLow = true;
    return BELOW_THIRTY;
  }
}

/**  电量指示
 * @brief     用ws2812b不同颜色指示电量状态
 * @param     batteryLED: 电量指示灯状态标志位
 */
void batteryFlash(BatteryState state) {
  if (batteryLED) {
    switch (state) {
    case SEVENTY_PLUS:
      myRGB.setPixelColor(0, blue);
      break;
    case FIFTY_TO_SEVENTY:
      myRGB.setPixelColor(0, green);
      break;
    case THIRTY_TO_FIFTY:
      myRGB.setPixelColor(0, yellow);
      break;
    default:
      break;
    }
#if DEBUG
    Serial.println("/*****************************/");
    Serial.println("电池电量指示函数:");
    Serial.print("电池电压: ");
    Serial.print(battery._voltage);
    Serial.print("V, 电量百分比: ");
    Serial.print(battery._voltsPercentage);
    Serial.println("%");
#endif
  }
}

/**  电量读取指示和报警任务
 * @brief     电量指示灯管理：上电/按键唤醒显示30秒后自动休眠；
              低电量报警管理：低于设定阈值后循环报警
 * @param     R1: 分压电阻R1阻值
 * @param     R2: 分压电阻R2阻值
 */
void batteryCheck(void* pvParameter) {
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
  while (1) {
    // 未到低电量阈值时采用指示+休眠方式
    if (!batteryLow) {
      myRGB.clear();
      BatteryState  state         = batteryReading(); // 定期进行电量检测，所以不直接把batteryReading函数作为形参传入
      unsigned long flashOnMillis = millis();
      if (millis() - flashOnMillis < FLASH_INTERVAL) { // 判断是否唤醒显示
        batteryFlash(state);
      } else {
        myRGB.clear();
        batteryLED = false;
      }
    } else {
      // 根据电量百分比调整检测频率，电量越低检测越频繁
      int delayTime = BATTERY_READING_INTERVAL * battery._voltsPercentage / 100 / portTICK_PERIOD_MS;
      if (delayTime < 30 * 1000 / portTICK_PERIOD_MS) delayTime = 30 * 1000 / portTICK_PERIOD_MS; // 最小间隔30秒
      myRGB.setPixelColor(0, red);
      buzzer(5, LONG_BEEP_DURATION, LONG_BEEP_INTERVAL);
#if DEBUG
      Serial.print("!!! 低电量报警 !!! 电量: ");
      Serial.print(battery._voltsPercentage);
      Serial.println("%");
      Serial.println("/*****************************/");
      Serial.printf("下一次检测将在 %d 分钟后\n", delayTime * battery._voltsPercentage / 100 / 60000);
#endif
      vTaskDelay(delayTime);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); // 每次检测后延时0.5秒
  }
}

// usb状态改变中断处理函数
void IRAM_ATTR handleUSBInterrupt() {
  usbConnected = digitalRead(USB_PIN);
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

void esp_now_connection(void* pvParameter) {
  while (1) {
    unsigned long currentTime = millis();
    esp_now_connected         = (currentTime - lastSendTime <= CONNECTION_TIMEOUT);
#if DEBUG
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime > 2000) { // 每2秒打印一次，避免刷屏
      if (esp_now_connected && recvSucceed) {
        Serial.println("连接检测任务：ESP NOW 接收发送正常！");
      } else if (esp_now_connected && !recvSucceed) {
        Serial.println("连接检测任务：ESP NOW 接收成功，但发送异常！");
      } else if (!esp_now_connected && recvSucceed) {
        Serial.println("连接检测任务：ESP NOW 接收超时，但发送成功！");
      } else if (!esp_now_connected && !recvSucceed) {
        Serial.println("连接检测任务：ESP NOW 彻底断线！");
      }
      lastDebugTime = currentTime;
    }
#endif
    if (esp_now_connected == true && usbConnected == false) {
      digitalWrite(RGB_LED_PIN, LOW); // 共阳极RBG，低电平点亮
    } else {
      digitalWrite(RGB_LED_PIN, HIGH);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/*-------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RGB_LED_PIN, OUTPUT);
  pinMode(WS2812_PIN, OUTPUT);

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