#include "OneButton.h"
#include "batteryReading.hpp"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define DEBUG 1

/*----------------------------------------------- ESP NOW-----------------------------------------------*/

// uint8_t BoosterAddress[] = { 0x9c, 0x13, 0x9e, 0x55, 0x1b, 0xa8 }; // 测试板
uint8_t BoosterAddress[] = { 0xdc, 0xda, 0x0c, 0xc2, 0x5e, 0x08 }; // super mini

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

bool esp_now_connected;

/*----------------------------------------- MPU6050 & QMC5883P -----------------------------------------*/

#define SDA_PIN 19
#define SCL_PIN 18

// float yaw, gyro_Z; // Z轴角度，Z轴角速度
// MPU6050 mpu6050(Wire);

/*----------------------------------------------- 操控 -----------------------------------------------*/

#define STEP_TURN_LEFT 5
#define STEP_TURN_RIGHT 0
#define THROTTLE_PIN 6
#define FUNCTION_PIN 7
#define STEP_SPEED 1

OneButton function;

/*---------------------------------------------- 蜂鸣器 ----------------------------------------------*/

#define BUZZER_PIN 10
#define LONG_BEEP_DURATION 1000
#define SHORT_BEEP_DURATION 200
#define LONG_BEEP_INTERVAL 300
#define SHORT_BEEP_INTERVAL 100

/*----------------------------------------------- WS2812 -----------------------------------------------*/

#define WS2812_PIN 8
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0
#define STANDARD_BRIGHTNESS 100
#define SHORT_FLASH_DURATION 200
#define SHORT_FLASH_INTERVAL 200
#define LONG_FLASH_DURATION 500
#define LONG_FLASH_INTERVAL 500

uint16_t brightness; // 动态亮度

Adafruit_NeoPixel myRGB(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
uint32_t          red    = myRGB.Color(255, 0, 0);  // 红色
uint32_t          green  = myRGB.Color(0, 255, 0);  // 绿色
uint32_t          blue   = myRGB.Color(0, 0, 255);  // 蓝色
uint32_t          yellow = myRGB.Color(255, 80, 0); // 黄色

/*----------------------------------------------- 电池电量 -----------------------------------------------*/

#define BATTERY_PIN 4
#define BATTERY_MAX_VALUE 4.2                  // 电池最大电量
#define BATTERY_MIN_VALUE 3.2                  // 电池最小电量
#define BATTERY_MIN_PERCENTAGE 20              // 电池最低百分比
#define BATTERY_READING_AVERAGE 50             // 采样次数
#define BATTERY_READING_INTERVAL 3 * 60 * 1000 // 采样间隔 5分钟*60秒*1000毫秒
#define R1 10000
#define R2 9950

enum BatteryState {
  SEVENTY_PLUS,     // 70%以上
  FIFTY_TO_SEVENTY, // 50%-70%
  THIRTY_TO_FIFTY,  // 30%-50%
  BATTERY_LOW       // 30%以下
};
BatteryState batteryState;
BatReading   battery;

float batvolts, voltsPercentage;

bool lowBatteryAlerted = false; // 低电量告警标志位

/*----------------------------------------------- 自定义函数 -----------------------------------------------*/
// 数据发出去之后的回调函数
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
#if DEBUG
  status == ESP_NOW_SEND_SUCCESS ? Serial.println("数据发送回调函数：数据发送成功") : Serial.println("数据发送回调函数：数据发送失败");
#endif
}

// 收到消息后的回调
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&booster, incomingData, sizeof(booster));
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

// 电量开机初检
void batteryFirstCheck() {
  // 开机执行一次，提示电池电量
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE); // 引脚、R1阻值、R2阻值、最大电压、最小（报警）电压
  battery.readMilliVolts(BATTERY_READING_AVERAGE);
#if DEBUG
  Serial.print("电池电压: ");
  Serial.print(battery._voltage);
  Serial.print("V, 电量百分比: ");
  Serial.print(battery._voltsPercentage);
  Serial.println("%");
#endif
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
#if DEBUG
    Serial.println("esp now初始化函数：ESP NOW 初始化成功");
#endif
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
  } else {
    // 报警
    myRGB.clear();
    myRGB.setPixelColor(0, red);
    myRGB.show();
    // 尝试重试3次
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
#if DEBUG
      Serial.println("ESP NOW 重连失败");
#endif
    }
  }
}

// 功能按键回调函数
void functionButton() {
  footPad.stepData[3] = !footPad.stepData[3];
  buzzer(1, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
#if DEBUG
  if (footPad.stepData[3]) {
    Serial.println("功能键按下");
  }
#endif
}

// 电量读取
BatteryState batteryReading() {
  battery.readMilliVolts(BATTERY_READING_AVERAGE);
  voltsPercentage = battery._voltsPercentage;
  batvolts        = battery._voltage;
  if (voltsPercentage >= 70)
    return SEVENTY_PLUS;
  else if (voltsPercentage >= 50)
    return FIFTY_TO_SEVENTY;
  else if (voltsPercentage >= 30)
    return THIRTY_TO_FIFTY;
  else
    return BATTERY_LOW;
#if DEBUG
  Serial.print("电池电压: ");
  Serial.print(battery._voltage);
  Serial.print("V, 电量百分比: ");
  Serial.print(battery._voltsPercentage);
  Serial.println("%");
#endif
}

// 电池电量提示与报警
void batteryCheck(void* pvParameter) {
  while (1) {
    int delayTime = BATTERY_READING_INTERVAL * voltsPercentage / 100 / portTICK_PERIOD_MS; // 根据电量百分比调整检测频率，电量越低检测越频繁
    if (delayTime < 1 * 60 * 1000 / portTICK_PERIOD_MS) {                                  // 最小间隔1分钟
      delayTime = 1 * 60 * 1000 / portTICK_PERIOD_MS;
    }
    switch (batteryReading()) {
    case SEVENTY_PLUS:
      myRGB.setPixelColor(0, blue);
      break;
    case FIFTY_TO_SEVENTY:
      myRGB.setPixelColor(0, green);
      break;
    case THIRTY_TO_FIFTY:
      myRGB.setPixelColor(0, yellow);
      break;
    case BATTERY_LOW:
      myRGB.setPixelColor(0, red);
      if (!lowBatteryAlerted) {
        buzzer(3, SHORT_BEEP_DURATION, SHORT_BEEP_INTERVAL);
        lowBatteryAlerted = true;
      } else {
        lowBatteryAlerted = false;
      }
      vTaskDelay(delayTime);
      break;
    default:
      break;
    }
    myRGB.show();
  }
}

// 数据发送任务
void dataTransmit(void* pvParameter) {
  function.setup(FUNCTION_PIN, INPUT_PULLUP);
  function.attachLongPressStart(functionButton);
  function.setPressMs(600);
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(12.5); // 频率 80Hz → 周期为 1/80 = 0.0125 秒 = 12.5 毫秒
  while (1) {
    function.tick();
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
/*-------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(WS2812_PIN, OUTPUT);

  pinMode(STEP_TURN_LEFT, INPUT_PULLUP);
  pinMode(STEP_TURN_RIGHT, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(FUNCTION_PIN, INPUT_PULLUP);

  analogReadResolution(12);

  esp_now_connect();
  batteryFirstCheck();

  xTaskCreate(dataTransmit, "dataTransmit", 1024 * 2, NULL, 1, NULL);
  xTaskCreate(batteryCheck, "batteryCheck", 1024 * 2, NULL, 1, NULL);

#if DEBUG
  Serial.println("脚控初始化完成");
#endif
}
void loop() {
  esp_now_connected == true ? Serial.println("ESP-NOW 连接成功") : Serial.println("ESP-NOW 连接失败");
  delay(1000);
}