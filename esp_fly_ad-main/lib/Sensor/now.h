#ifndef NOW_H
#define NOW_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "ESPNowW.h"
#include "utility.h"

const int8_t OFFSET_lx = -38;  // 摇杆校准偏移值
const int8_t OFFSET_ly = -21;
const int8_t OFFSET_rx = -15;
const int8_t OFFSET_ry = -21;

const int8_t DEAD_AREA = 20;  // 摇杆死区

class ESPNowReceiver {
public:
    ESPNowReceiver(); // 默认构造函数
    ESPNowReceiver(uint8_t* mac);
    void begin();
    int* getParsedData();
    int* getParsedDataFix();
    int getParsedDataSize();
    const uint8_t* getLastMacAddr(); // 获取最后接收到的MAC地址
    bool isSignalLost(); // 检测信号是否丢失

private:
    static void onReceiveStatic(const uint8_t* mac_addr, const uint8_t* data, int data_len); // 添加静态成员函数声明
    void onReceive(const uint8_t* mac_addr, const uint8_t* data, int data_len);
    static ESPNowReceiver* instance; // 声明静态成员变量

    uint8_t* mac;
    int parsedData[8];     // 原始数据
    int parsedDataFix[8];  // 修正后的数据
    int parsedDataSize;
    uint8_t lastMacAddr[6]; // 存储最后接收到的MAC地址

    unsigned long lastReceiveTime; // 存储最后接收到数据的时间
    const unsigned long signalTimeout = 1000; // 信号丢失超时时间（毫秒）
    
    hw_timer_t *timer; // 添加定时器成员变量
    static void IRAM_ATTR timerCallback(); // 添加定时器回调函数声明
};

#endif // NOW_H