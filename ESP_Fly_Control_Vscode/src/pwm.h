#ifndef PWM_H
#define PWM_H

// 引入必要的库（根据实际使用的平台补充，例如ESP32的PWM库）
#include "driver/ledc.h"  // 以ESP32为例，使用ledc相关函数需要包含此头文件
#define FREQ       50   //频率
#define CHANNEL0       0   //通道
#define CHANNEL1       1   //通道
#define CHANNEL2       2   //通道
#define CHANNEL3       3   //通道
#define RESOLUTION     13  // 提高到12位分辨率（0-4095范围）
#define PIN0           27   //引脚 
#define PIN1           13   //引脚 
#define PIN2           12   //引脚 
#define PIN3           26   //引脚 
// 声明PWM初始化函数
void PWM_init(void);
extern float target_Y_speed,target_X_speed;

#endif  