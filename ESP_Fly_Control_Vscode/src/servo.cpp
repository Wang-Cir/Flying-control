#include "head.h"

// UART2 配置（JY61P通常使用115200波特率）
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17
#define UART2_BAUD_RATE 115200

HardwareSerial SerialPort(2);

// JY61P数据接收变量（按照官方代码定义）
static uint8_t RxBuffer[11], RxData;
static volatile uint8_t RxState = 0;
static uint8_t RxIndex = 0;
static uint8_t CurrentPacketType = 0; // 当前数据包类型

//角度数据
float Roll, Pitch, Yaw;
//角速度数据
float gyroX, gyroY, gyroZ;


// 数据包计数和错误计数
uint32_t packetCount = 0;
uint32_t errorCount = 0;
uint32_t anglePacketCount = 0;
uint32_t accelPacketCount = 0;


//串口中断初始化
void servo_init()
{
   // 初始化UART2
    SerialPort.begin(UART2_BAUD_RATE, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
    SerialPort.setRxBufferSize(256);
    SerialPort.onReceive(onUART2Data);
}

/**
 * @brief JY61P数据包处理函数（支持0x51加速度和0x53角度数据包）
 * @param RxData - 接收到的单个字节
 */
void jy61p_ReceiveData(uint8_t RxData)
{
    uint8_t i, sum = 0;
    
    if (RxState == 0)    // 等待包头0x55
    {
        if (RxData == 0x55) // 收到包头
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 1;
            RxIndex = 1;
        }
    }
    else if (RxState == 1) // 等待数据标识0x51（加速度）或0x53（角度）
    {
        if (RxData == 0x52 || RxData == 0x53) // 加速度或角度数据包
        {
            RxBuffer[RxIndex] = RxData;
            CurrentPacketType = RxData; // 保存当前数据包类型
            RxState = 2;
            RxIndex = 2;
        }
        else
        {
            // 不是期望的数据包，重置状态
            RxState = 0;
            RxIndex = 0;
        }
    }
    else if (RxState == 2) // 接收数据部分
    {
        RxBuffer[RxIndex++] = RxData;
        if (RxIndex == 11) // 接收完成（11字节数据包）
        {
            // 计算校验和
            for (i = 0; i < 10; i++)
            {
                sum = sum + RxBuffer[i];
            }
            
            if (sum == RxBuffer[10]) // 校验成功
            {
                // 根据数据包类型解析数据
                if (CurrentPacketType == 0x53) // 角度数据包
                {
                    // 解析角度数据（按照官方公式）
                    Roll = ((int16_t)((uint16_t)RxBuffer[3] << 8 | (uint16_t)RxBuffer[2])) / 32768.0f * 180.0f;
                    Pitch = ((int16_t)((uint16_t)RxBuffer[5] << 8 | (uint16_t)RxBuffer[4])) / 32768.0f * 180.0f;
                    Yaw = ((int16_t)((uint16_t)RxBuffer[7] << 8 | (uint16_t)RxBuffer[6])) / 32768.0f * 180.0f;
                    anglePacketCount++;
                }
                else if (CurrentPacketType == 0x52)  //角速度数据包
                {
                    // 解析加速度数据（按照官方公式，量程通常为±16g）
                    gyroX = ((int16_t)((uint16_t)RxBuffer[3] << 8 | (uint16_t)RxBuffer[2])) / 32768.0f * 2000.0f;
                    gyroY = ((int16_t)((uint16_t)RxBuffer[5] << 8 | (uint16_t)RxBuffer[4])) / 32768.0f * 2000.0f;
                    gyroZ = ((int16_t)((uint16_t)RxBuffer[7] << 8 | (uint16_t)RxBuffer[6])) / 32768.0f * 2000.0f;
                    accelPacketCount++;
                }
                    
                
                packetCount++;
            }
            else
            {
                errorCount++;
            }
            
            // 重置接收状态
            RxState = 0;
            RxIndex = 0;
            CurrentPacketType = 0;
        }
    }
}

// UART2中断回调函数
void onUART2Data() {
    while (SerialPort.available()) {
        RxData = SerialPort.read();
        jy61p_ReceiveData(RxData);
    }
}

 float Roll_error2(float Target_jy62,float Actual_jy62) {
    // 归一化目标角度到0-360范围
    float target = fmod(Target_jy62, 360);
    if (target < 0) {
        target += 360;
    }

    // 归一化实际角度到0-360范围
    float actual = fmod(Actual_jy62, 360);
    if (actual < 0) {
        actual += 360;
    }

    // 计算原始误差
    float error = target - actual;

    // 调整误差到[-180, 180)范围，确保最短路径
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    return error;
}

 float Pitch_error2(float Target_jy62,float Actual_jy62) {
    // 归一化目标角度到0-360范围
    float target = fmod(Target_jy62, 360);
    if (target < 0) {
        target += 360;
    }

    // 归一化实际角度到0-360范围
    float actual = fmod(Actual_jy62, 360);
    if (actual < 0) {
        actual += 360;
    }

    // 计算原始误差
    float error = target - actual;

    // 调整误差到[-180, 180)范围，确保最短路径
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    return error;
}

 float Yaw_error2(float Target_jy62,float Actual_jy62) {
    // 归一化目标角度到0-360范围
    float target = fmod(Target_jy62, 360);
    if (target < 0) {
        target += 360;
    }

    // 归一化实际角度到0-360范围
    float actual = fmod(Actual_jy62, 360);
    if (actual < 0) {
        actual += 360;
    }

    // 计算原始误差
    float error = target - actual;

    // 调整误差到[-180, 180)范围，确保最短路径
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    return error;
}