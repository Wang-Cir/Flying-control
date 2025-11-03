#ifndef _PID_H_  // 头文件保护（防止重复包含，命名规则：_项目名_模块名_H_）
#define _PID_H_

// 1. 包含必要依赖（根据你的工程调整）
#include "head.h"  // 你的主头文件（包含MPU6050、蓝牙等声明）
#include <math.h>  // 提供abs()等数学函数

extern int throttle,DIR_UP;           // 油门基准值（0-410，与你的PWM匹配）
extern float roll_angle;       // 当前滚转角（来自MPU6050）
extern float pitch_angle;      // 当前俯仰角
extern float target_roll;      // 目标滚转角（遥控器/自稳模式给定）
extern float target_pitch;     // 目标俯仰角
extern int motor_output[4];     // 电机PWM输出（0-410）
extern float P_roll;
extern float roll_rate_output, pitch_rate_output;
// extern float roll_rate_integral, pitch_rate_integral;
// 4. 核心控制函数声明（实现位于pid.cpp）
void PID_Control();  // 500Hz定时器中断调用的PID计算函数

extern float target_roll_rate, target_pitch_rate, target_yaw_rate;
extern float integral_pitch,integral_roll;
#endif  // _PID_H_