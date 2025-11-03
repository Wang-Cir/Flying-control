#include <Arduino.h>
#include "motors.h"
#include "imu.h"

IMUClass imu;  // 使用自定义引脚 IMUClass imu(10, 11, 12, 13);

// QuadMotorController quadMotorController; // 使用自定义引脚 
QuadMotorController motors(18, 16, 21, 17);

void setup() {
    Serial.begin(115200);
    while (!Serial) {} // 等待串口连接

    // 设置电机推力
    int thr_list[4] = {200, 300, 400, 500};
    motors.setMotorsThr(thr_list);

    // 获取并打印电机推力
    int thr_list_read[4];
    motors.getMotorsThr(thr_list_read);
    Serial.println("Motor Thrust Values:");
    for (int i = 0; i < 4; i++) {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(thr_list_read[i]);
    }

    // 等待一段时间
    int wait_s = 5;
    for (int i = 0; i < wait_s; i++) {
        Serial.printf("Wait for %d seconds...\n", i);
        delay(1000);
    }

    // 设置相对推力
    int thr_relative_list[4] = {50, -1000, -2000, -600};
    motors.setMotorsThrRelative(thr_relative_list);

    // 获取并打印电机推力
    motors.getMotorsThr(thr_list_read);
    Serial.println("Motor Thrust Values after Relative Adjustment:");
    for (int i = 0; i < 4; i++) {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(thr_list_read[i]);
    }

    // 等待一段时间
    wait_s = 5;
    for (int i = 0; i < wait_s; i++) {
        Serial.printf("Wait for %d seconds...\n", i);
        delay(1000);
    }

    // 重置电机推力
    motors.reset();

    // 获取并打印电机推力
    motors.getMotorsThr(thr_list_read);
    Serial.println("Motor Thrust Values after Reset:");
    for (int i = 0; i < 4; i++) {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(thr_list_read[i]);
    }
}

void loop() {
    // 主循环中不需要执行任何操作
}