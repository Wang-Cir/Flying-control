#include <Arduino.h>

void Task1(void *pvParameters) {
    for (;;) {
        // 执行任务1的代码
        Serial.println("Task 1 is running");
        delay(1000); // 延迟1秒
    }
}

void Task2(void *pvParameters) {
    for (;;) {
        // 执行任务2的代码
        Serial.println("Task 2 is running");
        delay(500); // 延迟0.5秒
    }
}