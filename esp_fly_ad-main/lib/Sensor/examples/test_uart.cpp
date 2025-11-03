#include <Arduino.h>
#include <QuickPID.h>

#include "imu.h"
#include "now.h"
#include "motors.h"
#include "config.h"
#include "utility.h"

#include <Arduino.h>
#include "uart.h"

// 创建 UART 对象
UART myUART;

// 定义 PID 参数
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

void setup() {
    // 初始化串口
    myUART.begin(Serial, 115200);

    // 示例：发送一些数据到串口
    Serial.println("请输入 PID 参数，格式为：kp:1.0\nki:0.1\nkd:0.01");
}

void loop() {
    // 更新 PID 参数
    myUART.UpdatePidParams(kp, ki, kd);

    // 打印当前的 PID 参数
    Serial.printf("Current PID Parameters - kp: %f, ki: %f, kd: %f\n", kp, ki, kd);

    // 延迟一段时间
    delay(500);
}