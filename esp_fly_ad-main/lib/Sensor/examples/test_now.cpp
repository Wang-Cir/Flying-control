#include <Arduino.h>
#include "now.h"
#include "imu.h"

static uint8_t defaultMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 默认广播地址
ESPNowReceiver receiver(defaultMac); // 使用默认构造函数

void setup() {
    Serial.begin(115200);
    delay(1000); // 等待串口打开
    receiver.begin();
    Serial.println("Test Setup Complete");
}

void loop() {
    // 获取解析后的数据
    int* parsedData = receiver.getParsedData();
    int parsedDataSize = receiver.getParsedDataSize();
    const uint8_t* lastMacAddr = receiver.getLastMacAddr();

    // 打印解析后的数据
    Serial.print("Parsed Data Size: ");
    Serial.println(parsedDataSize);

    Serial.print("Parsed Data: ");
    for (int i = 0; i < parsedDataSize; i++) {
        Serial.print(parsedData[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.println("Last MAC Address:");
    for (int i = 0; i < 6; i++) {
        Serial.print(lastMacAddr[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // 延迟一段时间后再次测试
    delay(10);
}