#include "OneButton.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define DEBUG 0

/*----------------------------------------------- ESP NOW-----------------------------------------------*/

uint8_t BoosterAddress[6];

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

// 接收的数据
struct Booster {
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

/*----------------------------------------------- RGB LED-----------------------------------------------*/

#define RGB_LED_PIN 2
#define LONG_BLINK_DURATION 1000
#define SHORT_BLINK_DURATION 200
#define LONG_BLINK_INTERVAL 300
#define SHORT_BLINK_INTERVAL 100

/*----------------------------------------------- 电池电量 -----------------------------------------------*/

#define BATTERY_PIN 4
#define BATTERY_MAX_VALUE 4.2             // 电池最大电量
#define BATTERY_MIN_VALUE 3.2             // 电池最小电量
#define BATTERY_MIN_PERCENTAGE 20         // 电池最低百分比
#define PAD_BATTERY_READING_INTERVAL 3000 // 采样间隔
#define R1 10000
#define R2 9950

/*----------------------------------------------- 自定义函数 -----------------------------------------------*/

// 数据发出去之后的回调函数
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // 如果发送成功
  if (status == ESP_NOW_SEND_SUCCESS) {
    esp_now_connected = true;
#if DEBUG
    Serial.println("数据发送成功");
#endif
  } else {
    esp_now_connected = false;
#if DEBUG
    Serial.println("数据发送失败");
#endif
  }
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
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数

  // 注册通信频道
  memcpy(peerInfo.peer_addr, BoosterAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                          // 设置通信频道
  esp_now_add_peer(&peerInfo);                   // 添加通信对象

  // 如果初始化失败则重连
  while (esp_now_init() != ESP_OK) {
#if DEBUG
    Serial.println("ESP NOW 初始化失败，正在重连...");
#endif
    // 报警

    /*LED 代码*/

    buzzer(3, LONG_BEEP_DURATION, LONG_BEEP_INTERVAL);
    // 重连
    esp_now_init();                                // 初始化ESP NOW
    esp_now_register_send_cb(OnDataSent);          // 注册发送成功的回调函数
    esp_now_register_recv_cb(OnDataRecv);          // 注册接受数据后的回调函数
    memcpy(peerInfo.peer_addr, BoosterAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
    peerInfo.channel = 1;                          // 设置通信频道
    esp_now_add_peer(&peerInfo);                   // 添加通信对象

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
// 初始化成功
#if DEBUG
  Serial.println("ESP NOW 初始化成功");
#endif
  // LED相关代码
}

void functionButton() { }
/*-------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RGB_LED_PIN, OUTPUT);

  pinMode(STEP_TURN_LEFT, INPUT_PULLUP);
  pinMode(STEP_TURN_RIGHT, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(FUNCTION_PIN, INPUT_PULLUP);

  analogReadResolution(12);
  esp_now_connect();
  function.setup(FUNCTION_PIN, INPUT_PULLUP);
  function.attachDoubleClick(functionButton);
}
void loop() {
}
