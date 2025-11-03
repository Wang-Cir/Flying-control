#ifndef SERVO_H
#define SERVO_H

#include <HardwareSerial.h>

// UART2 配置（JY61P通常使用9600波特率）
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17
#define UART2_BAUD_RATE 115200

// 外部声明全局变量，供其他文件使用
extern float Roll;
extern float Pitch;
extern float Yaw;

extern float gyroX, gyroY, gyroZ;


extern uint32_t packetCount;
extern uint32_t errorCount;

// 函数声明
void servo_init();
void jy61p_ReceiveData(uint8_t RxData);
void onUART2Data();

// 定义四个角的高度值（全局变量或根据需求调整）
extern volatile float height_FL, height_FR, height_BL, height_BR;

float Roll_error2(float Target_jy62,float Actual_jy62);
float Pitch_error2(float Target_jy62,float Actual_jy62);
float Yaw_error2(float Target_jy62,float Actual_jy62);

#endif 